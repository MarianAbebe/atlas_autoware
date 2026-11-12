#include "atlas_nav2_demo/ackermann_controller.hpp"
#include "atlas_nav2_demo/ackermann_controller_math.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <sstream>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/qos.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace atlas_nav2_demo
{
namespace
{
constexpr double kEpsilon = 1.0e-9;
using controller_math::boundedStep;
using controller_math::purePursuitCurvature;
using controller_math::steeringFromCurvature;
using controller_math::sourceStampFresh;
using controller_math::wrapAngle;

struct PlanPoint
{
  double x;
  double y;
  double yaw;
};

int directionOf(const PlanPoint & a, const PlanPoint & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dyaw = wrapAngle(b.yaw - a.yaw);
  const double mid_yaw = wrapAngle(a.yaw + 0.5 * dyaw);
  const double longitudinal = dx * std::cos(mid_yaw) + dy * std::sin(mid_yaw);
  if (std::hypot(dx, dy) < 1.0e-5 || std::abs(longitudinal) < 1.0e-5) {
    return 0;
  }
  return longitudinal > 0.0 ? 1 : -1;
}
}  // namespace

void AckermannController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  node_ = parent_.lock();
  if (!node_) {
    throw std::runtime_error("Ackermann controller parent lifecycle node expired");
  }
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  auto parameter = [this](const std::string & key, double fallback) {
      return node_->declare_parameter<double>(plugin_name_ + "." + key, fallback);
    };
  wheelbase_ = parameter("wheelbase_m", 1.20);
  steering_limit_ = parameter("steering_limit_rad", 0.50);
  steering_rate_limit_ = parameter("steering_rate_limit_radps", 0.60);
  acceleration_limit_ = parameter("acceleration_limit_mps2", 0.80);
  deceleration_limit_ = parameter("deceleration_limit_mps2", 1.20);
  max_speed_ = parameter("max_speed_mps", 0.35);
  lookahead_ = parameter("lookahead_m", 1.00);
  minimum_lookahead_ = parameter("minimum_lookahead_m", 0.50);
  lookahead_step_ = parameter("lookahead_search_step_m", 0.10);
  controller_period_ = parameter("controller_period_s", 0.10);
  reaction_time_ = parameter("reaction_time_s", 0.37);
  stopping_buffer_ = parameter("stopping_buffer_m", 0.05);
  collision_step_ = parameter("collision_check_step_m", 0.05);
  const auto scan_topic = node_->declare_parameter<std::string>(
    plugin_name_ + ".obstacle_scan_topic", "/atlas_ackermann/scan");
  obstacle_scan_timeout_ = parameter("obstacle_scan_timeout_s", 0.30);
  goal_position_tolerance_ = parameter("goal_position_tolerance_m", 0.10);
  goal_heading_tolerance_ = parameter("goal_heading_tolerance_rad", 0.12);
  cusp_stop_tolerance_ = parameter("cusp_stop_tolerance_m", 0.04);
  speed_limit_ = max_speed_;

  if (wheelbase_ <= 0.0 || steering_limit_ <= 0.0 || steering_limit_ >= M_PI_2 ||
    steering_rate_limit_ <= 0.0 || acceleration_limit_ <= 0.0 ||
    deceleration_limit_ <= 0.0 || max_speed_ <= 0.0 || lookahead_ <= 0.0 ||
    minimum_lookahead_ <= 0.0 || minimum_lookahead_ > lookahead_ ||
    lookahead_step_ <= 0.0 ||
    controller_period_ <= 0.0 || reaction_time_ < 0.0 ||
    stopping_buffer_ < 0.0 || collision_step_ <= 0.0 ||
    obstacle_scan_timeout_ <= 0.0)
  {
    throw std::invalid_argument("invalid Ackermann controller limits");
  }
  scan_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    std::bind(&AckermannController::onLaserScan, this, std::placeholders::_1));
  auto node_parameters = node_->get_node_parameters_interface();
  auto node_topics = node_->get_node_topics_interface();
  compute_trace_publisher_ = rclcpp::create_publisher<std_msgs::msg::String>(
    node_parameters, node_topics,
    "/atlas_ackermann/controller_compute_trace", rclcpp::QoS(10));
  RCLCPP_INFO(
    node_->get_logger(),
    "Ackermann controller configured: wheelbase %.2f m, steering %.3f rad, "
    "curvature %.4f 1/m, speed %.2f m/s, reaction %.2f s",
    wheelbase_, steering_limit_, std::tan(steering_limit_) / wheelbase_,
    max_speed_, reaction_time_);
}

void AckermannController::cleanup()
{
  scan_subscription_.reset();
  compute_trace_publisher_.reset();
  plan_.poses.clear();
  tf_.reset();
  costmap_ros_.reset();
  node_.reset();
  active_ = false;
}

void AckermannController::onLaserScan(
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  const int64_t source_ns =
    static_cast<int64_t>(scan->header.stamp.sec) * 1000000000LL +
    static_cast<int64_t>(scan->header.stamp.nanosec);
  const int64_t now_ns = node_ ? node_->now().nanoseconds() : 0;
  const int64_t timeout_ns = static_cast<int64_t>(obstacle_scan_timeout_ * 1.0e9);
  const bool valid_stamp = sourceStampFresh(source_ns, now_ns, timeout_ns);
  std::lock_guard<std::mutex> lock(scan_mutex_);
  have_valid_scan_ = valid_stamp;
  if (valid_stamp) {
    last_scan_source_ns_ = source_ns;
    last_scan_receive_ = std::chrono::steady_clock::now();
  }
}

bool AckermannController::obstacleInputFresh() const
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!have_valid_scan_) {
    return false;
  }
  const auto receipt_age = std::chrono::steady_clock::now() - last_scan_receive_;
  const int64_t now_ns = node_ ? node_->now().nanoseconds() : 0;
  const int64_t source_age_ns = now_ns - last_scan_source_ns_;
  const auto timeout_ns = static_cast<int64_t>(obstacle_scan_timeout_ * 1.0e9);
  return receipt_age <= std::chrono::duration<double>(obstacle_scan_timeout_) &&
         source_age_ns >= 0 && source_age_ns <= timeout_ns;
}

void AckermannController::activate()
{
  active_ = true;
  last_steering_ = 0.0;
  last_command_speed_ = 0.0;
}

void AckermannController::deactivate()
{
  active_ = false;
}

void AckermannController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
  active_segment_ = 0;
  active_direction_ = 0;
}

void AckermannController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    return;
  }
  speed_limit_ = percentage ? max_speed_ * speed_limit / 100.0 :
    std::min(max_speed_, speed_limit);
}

geometry_msgs::msg::TwistStamped AckermannController::zeroCommand(
  const std_msgs::msg::Header & header) const
{
  geometry_msgs::msg::TwistStamped output;
  output.header = header;
  output.twist = geometry_msgs::msg::Twist();
  return output;
}

bool AckermannController::footprintClear(
  double x, double y, double yaw,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  const double cost = collision_checker_.footprintCostAtPose(x, y, yaw, footprint);
  if (!std::isfinite(cost) || cost < 0.0 ||
    cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    last_collision_x_ = x;
    last_collision_y_ = y;
    last_collision_yaw_ = yaw;
    last_collision_cost_ = cost;
    return false;
  }
  return true;
}

bool AckermannController::rolloutClear(
  double x, double y, double yaw, double speed, double steering,
  double target_speed, double target_steering, double duration, double step,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double & minimum_cost, double command_delay)
{
  double elapsed = 0.0;
  while (elapsed < duration - kEpsilon) {
    const double until_switch = elapsed < command_delay ? command_delay - elapsed : step;
    const double dt = std::min({step, duration - elapsed, until_switch});
    const double previous_x = x;
    const double previous_y = y;
    const double previous_yaw = yaw;
    const bool waiting_for_command = elapsed < command_delay - kEpsilon;
    const double active_speed_target = waiting_for_command ? speed : target_speed;
    const double active_steering_target = waiting_for_command ? steering : target_steering;
    speed = boundedStep(speed, active_speed_target, acceleration_limit_,
                        deceleration_limit_, dt);
    steering = boundedStep(steering, active_steering_target,
                           steering_rate_limit_, steering_rate_limit_, dt);
    steering = std::clamp(steering, -steering_limit_, steering_limit_);
    const double yaw_rate = speed * std::tan(steering) / wheelbase_;
    x += speed * std::cos(yaw) * dt;
    y += speed * std::sin(yaw) * dt;
    yaw = wrapAngle(yaw + yaw_rate * dt);

    // Sample the swept footprint at no more than half a costmap cell or the
    // configured spatial interval, including orientation changes.
    const double travel = std::hypot(x - previous_x, y - previous_y);
    const double turn = std::abs(wrapAngle(yaw - previous_yaw));
    const double radius = std::max(0.1, costmap_ros_->getRobotRadius());
    const unsigned int samples = std::max(1U, static_cast<unsigned int>(std::ceil(
        std::max(travel / collision_step_, turn * radius / collision_step_))));
    for (unsigned int i = 1; i <= samples; ++i) {
      const double fraction = static_cast<double>(i) / samples;
      const double sample_x = previous_x + fraction * (x - previous_x);
      const double sample_y = previous_y + fraction * (y - previous_y);
      const double sample_yaw = wrapAngle(
        previous_yaw + fraction * wrapAngle(yaw - previous_yaw));
      const double cost = collision_checker_.footprintCostAtPose(
        sample_x, sample_y, sample_yaw, footprint);
      minimum_cost = std::min(minimum_cost, cost);
      if (!std::isfinite(cost) || cost < 0.0 ||
        cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        last_collision_x_ = sample_x;
        last_collision_y_ = sample_y;
        last_collision_yaw_ = sample_yaw;
        last_collision_cost_ = cost;
        return false;
      }
    }
    elapsed += dt;
  }
  return true;
}

geometry_msgs::msg::TwistStamped AckermannController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto callback_start = std::chrono::steady_clock::now();
  auto compute = [&]() -> geometry_msgs::msg::TwistStamped {
  auto header = pose.header;
  if (node_) {
    header.stamp = node_->now();
  }
  if (!active_ || !node_ || plan_.poses.size() < 2 || !costmap_ros_ ||
    !tf_ || !costmap_ros_->isCurrent())
  {
    return zeroCommand(header);
  }

  const double now = node_->now().seconds();
  const double pose_stamp = rclcpp::Time(pose.header.stamp).seconds();
  if (!std::isfinite(pose_stamp) || pose_stamp <= 0.0 ||
    now < pose_stamp || now - pose_stamp > 0.30)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ackermann controller stopping on stale/invalid pose");
    return zeroCommand(header);
  }
  if (!obstacleInputFresh()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Ackermann controller stopping on missing or stale obstacle scan");
    last_command_speed_ = 0.0;
    return zeroCommand(header);
  }
  if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
    !std::isfinite(velocity.linear.x) || !std::isfinite(velocity.angular.z))
  {
    return zeroCommand(header);
  }

  std::vector<PlanPoint> points;
  points.reserve(plan_.poses.size());
  try {
    for (const auto & source : plan_.poses) {
      geometry_msgs::msg::PoseStamped transformed;
      if (source.header.frame_id == global_frame_) {
        transformed = source;
      } else {
        transformed = tf_->transform(source, global_frame_,
                                     tf2::durationFromSec(0.10));
      }
      const auto & p = transformed.pose.position;
      const double yaw = tf2::getYaw(transformed.pose.orientation);
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw)) {
        return zeroCommand(header);
      }
      points.push_back({p.x, p.y, yaw});
    }
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ackermann controller stopping on plan TF error: %s",
                         error.what());
    return zeroCommand(header);
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double measured_speed = velocity.linear.x;
  double current_steering = last_steering_;
  if (std::abs(measured_speed) >= 0.05) {
    current_steering = steeringFromCurvature(
      controller_math::curvatureFromTwist(measured_speed, velocity.angular.z),
      wheelbase_);
  }
  current_steering = std::clamp(current_steering, -steering_limit_, steering_limit_);

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return zeroCommand(header);
  }
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
    *(costmap->getMutex()));
  collision_checker_.setCostmap(costmap);
  auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.empty()) {
    return zeroCommand(header);
  }
  nav2_costmap_2d::padFootprint(footprint, stopping_buffer_);

  // Validate a latency-plus-braking envelope before issuing any motion. During
  // the reaction interval, assume the previous speed/steering command persists.
  double minimum_cost = 0.0;
  if (!footprintClear(x, y, yaw, footprint) ||
    !rolloutClear(x, y, yaw, measured_speed, current_steering,
      0.0, current_steering,
      reaction_time_ + std::abs(measured_speed) / deceleration_limit_ +
      controller_period_, collision_step_, footprint, minimum_cost,
      reaction_time_))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Stopping: costmap collision in braking envelope at (%.3f, %.3f, %.3f), cost %.1f; current speed %.3f m/s",
      last_collision_x_, last_collision_y_, last_collision_yaw_,
      last_collision_cost_, measured_speed);
    last_command_speed_ = 0.0;
    return zeroCommand(header);
  }

  std::vector<int> directions(points.size() - 1);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    directions[i] = directionOf(points[i], points[i + 1]);
    if (directions[i] == 0) {
      return zeroCommand(header);
    }
  }

  std::size_t selected = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  double selected_progress = 0.0;
  const int velocity_direction = std::abs(measured_speed) >= 0.05 ?
    (measured_speed > 0.0 ? 1 : -1) : active_direction_;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    if (velocity_direction != 0 && directions[i] != velocity_direction) {
      continue;
    }
    const double dx = points[i + 1].x - points[i].x;
    const double dy = points[i + 1].y - points[i].y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-10) {
      continue;
    }
    const double fraction = std::clamp(
      ((x - points[i].x) * dx + (y - points[i].y) * dy) / length2,
      0.0, 1.0);
    const double projected_x = points[i].x + fraction * dx;
    const double projected_y = points[i].y + fraction * dy;
    const double error = std::hypot(x - projected_x, y - projected_y);
    if (error < best_distance) {
      best_distance = error;
      selected = i;
      selected_progress = fraction * std::sqrt(length2);
    }
  }
  if (std::isfinite(best_distance) && std::abs(measured_speed) < 0.05 &&
    active_direction_ != 0 && directions[selected] == active_direction_)
  {
    std::size_t end = selected;
    double remaining_to_cusp = std::hypot(
      points[selected + 1].x - points[selected].x,
      points[selected + 1].y - points[selected].y) - selected_progress;
    while (end + 1 < directions.size() &&
      directions[end + 1] == active_direction_)
    {
      ++end;
      remaining_to_cusp += std::hypot(
        points[end + 1].x - points[end].x,
        points[end + 1].y - points[end].y);
    }
    if (end + 1 < directions.size() &&
      remaining_to_cusp <= cusp_stop_tolerance_ + 0.02)
    {
      selected = end + 1;
      selected_progress = 0.0;
    }
  }
  if (!std::isfinite(best_distance) || best_distance > 0.75) {
    last_command_speed_ = 0.0;
    return zeroCommand(header);
  }
  if (active_direction_ != 0 && directions[selected] != active_direction_ &&
    std::abs(measured_speed) > 0.02)
  {
    return zeroCommand(header);
  }
  active_segment_ = selected;
  active_direction_ = directions[selected];

  std::size_t run_end = selected;
  while (run_end + 1 < directions.size() &&
    directions[run_end + 1] == active_direction_)
  {
    ++run_end;
  }
  double progress = selected_progress;
  for (std::size_t i = selected; i > 0; --i) {
    if (directions[i - 1] != active_direction_) {
      break;
    }
    progress += std::hypot(points[i].x - points[i - 1].x,
                           points[i].y - points[i - 1].y);
  }
  double run_length = progress;
  for (std::size_t i = selected; i <= run_end; ++i) {
    run_length += std::hypot(points[i + 1].x - points[i].x,
                             points[i + 1].y - points[i].y);
  }
  const double distance_to_cusp = std::max(0.0, run_length - progress);
  const bool has_cusp = run_end + 1 < directions.size();
  auto targetAt = [&](double path_distance) {
      double remaining = std::min(run_length, path_distance) - progress;
      PlanPoint target = points[selected];
      const double first_length = std::hypot(
        points[selected + 1].x - points[selected].x,
        points[selected + 1].y - points[selected].y);
      const double fraction = selected_progress / std::max(1.0e-9, first_length);
      target.x += fraction * (points[selected + 1].x - points[selected].x);
      target.y += fraction * (points[selected + 1].y - points[selected].y);
      target.yaw = wrapAngle(points[selected].yaw + fraction *
        wrapAngle(points[selected + 1].yaw - points[selected].yaw));
      for (std::size_t i = selected; i <= run_end && remaining > 0.0; ++i) {
        const double length = std::hypot(points[i + 1].x - points[i].x,
                                         points[i + 1].y - points[i].y);
        const double start_along = i == selected ? selected_progress : 0.0;
        const double available = length - start_along;
        const double take = std::min(remaining, available);
        if (take > 0.0 && length > 1.0e-9) {
          const double f = std::clamp((start_along + take) / length, 0.0, 1.0);
          target.x = points[i].x + f * (points[i + 1].x - points[i].x);
          target.y = points[i].y + f * (points[i + 1].y - points[i].y);
          target.yaw = wrapAngle(points[i].yaw + f *
            wrapAngle(points[i + 1].yaw - points[i].yaw));
        }
        remaining -= take;
      }
      return target;
    };
  const double max_curvature = std::tan(steering_limit_) / wheelbase_;
  PlanPoint target{};
  double curvature = std::numeric_limits<double>::quiet_NaN();
  double requested_steering = std::numeric_limits<double>::quiet_NaN();
  bool feasible_preview = false;
  for (double preview = minimum_lookahead_;
    preview < lookahead_ + lookahead_step_ * 0.5;
    preview += lookahead_step_)
  {
    const auto candidate = targetAt(progress + preview);
    const double dx = candidate.x - x;
    const double dy = candidate.y - y;
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double candidate_curvature = purePursuitCurvature(local_x, local_y);
    const double candidate_steering = steeringFromCurvature(
      candidate_curvature, wheelbase_);
    if (std::isfinite(candidate_curvature) &&
      std::abs(candidate_curvature) <= max_curvature + 1.0e-9 &&
      std::abs(candidate_steering) <= steering_limit_ + 1.0e-9)
    {
      target = candidate;
      curvature = candidate_curvature;
      requested_steering = candidate_steering;
      feasible_preview = true;
      break;
    }
  }
  if (!feasible_preview) {
    last_command_speed_ = 0.0;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "No feasible Ackermann preview between %.2f and %.2f m",
      minimum_lookahead_, lookahead_);
    return zeroCommand(header);
  }
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = points.back().x;
  goal_pose.position.y = points.back().y;
  tf2::Quaternion goal_quaternion;
  goal_quaternion.setRPY(0.0, 0.0, points.back().yaw);
  goal_pose.orientation = tf2::toMsg(goal_quaternion);
  if (goal_checker && goal_checker->isGoalReached(
      pose.pose, goal_pose, velocity))
  {
    last_command_speed_ = 0.0;
    last_steering_ = 0.0;
    return zeroCommand(header);
  }

  double desired_speed = active_direction_ * speed_limit_;
  if (has_cusp) {
    desired_speed = std::copysign(std::min(std::abs(desired_speed),
      std::sqrt(2.0 * deceleration_limit_ * std::max(
        0.0, distance_to_cusp - cusp_stop_tolerance_))), active_direction_);
  }
  double bounded_speed = boundedStep(measured_speed, desired_speed,
                                    acceleration_limit_, deceleration_limit_,
                                    controller_period_);
  if (std::abs(bounded_speed) > 1.0e-6 && std::abs(bounded_speed) < 0.05) {
    const double minimum_start_speed = std::min(
      0.05, acceleration_limit_ * controller_period_);
    if (minimum_start_speed < 0.05 - 1.0e-9) {
      last_command_speed_ = 0.0;
      return zeroCommand(header);
    }
    bounded_speed = std::copysign(0.05, bounded_speed);
  }
  double bounded_steering = boundedStep(current_steering, requested_steering,
    steering_rate_limit_, steering_rate_limit_, controller_period_);
  bounded_steering = std::clamp(bounded_steering, -steering_limit_, steering_limit_);

  // Validate one controller interval at the proposed command as well as the
  // braking envelope above. Footprint padding includes the explicit buffer.
  if (!rolloutClear(x, y, yaw, measured_speed, current_steering,
      bounded_speed, bounded_steering, controller_period_, collision_step_,
      footprint, minimum_cost))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Stopping: proposed control interval collides at (%.3f, %.3f, %.3f), cost %.1f",
      last_collision_x_, last_collision_y_, last_collision_yaw_,
      last_collision_cost_);
    last_command_speed_ = 0.0;
    return zeroCommand(header);
  }

  geometry_msgs::msg::TwistStamped output;
  output.header = header;
  output.twist.linear.x = bounded_speed;
  output.twist.angular.z = std::abs(bounded_speed) < 0.05 ? 0.0 :
    bounded_speed * std::tan(bounded_steering) / wheelbase_;
  last_steering_ = bounded_steering;
  last_command_speed_ = bounded_speed;
  return output;
  };

  auto output = compute();
  const auto callback_end = std::chrono::steady_clock::now();
  const auto start_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    callback_start.time_since_epoch()).count();
  const auto end_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    callback_end.time_since_epoch()).count();
  if (compute_trace_publisher_) {
    std_msgs::msg::String trace;
    std::ostringstream data;
    data << "{\"sequence\":" << ++compute_trace_sequence_
         << ",\"callback_start_monotonic_ns\":" << start_ns
         << ",\"callback_end_monotonic_ns\":" << end_ns
         << ",\"callback_duration_ns\":" << (end_ns - start_ns)
         << ",\"output_linear_mps\":" << output.twist.linear.x
         << ",\"output_angular_radps\":" << output.twist.angular.z
         << ",\"pose_stamp_sec\":" << pose.header.stamp.sec
         << ",\"pose_stamp_nanosec\":" << pose.header.stamp.nanosec << "}";
    trace.data = data.str();
    compute_trace_publisher_->publish(trace);
  }
  return output;
}

}  // namespace atlas_nav2_demo

PLUGINLIB_EXPORT_CLASS(atlas_nav2_demo::AckermannController, nav2_core::Controller)
