#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class VehicleMotionDemo final : public rclcpp::Node
{
public:
  VehicleMotionDemo()
  : Node("vehicle_motion_demo"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    timeout_sec_ = declare_parameter<double>("timeout_sec", 12.0);
    hold_after_test_sec_ = declare_parameter<double>("hold_after_test_sec", 0.0);
    command_pub_ = create_publisher<autoware_control_msgs::msg::Control>(
      "input/ackermann_control_command", rclcpp::QoS(1));
    gear_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
      "input/gear_command", rclcpp::QoS(1));
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "visualization/vehicle", rclcpp::QoS(10));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "output/odometry", rclcpp::QoS(10), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        if (last_odom_stamp_.nanoseconds() != 0 && rclcpp::Time(msg->header.stamp) <= last_odom_stamp_) {
          fail("simulator odometry timestamps did not advance");
          return;
        }
        last_odom_stamp_ = rclcpp::Time(msg->header.stamp);
        ++odom_samples_;
        if (msg->header.frame_id != "atlas_sim_odom" || msg->child_frame_id != "atlas_sim_base_link") {
          fail("odometry frame IDs do not match the test-only simulator frames");
          return;
        }
        last_odom_ = *msg;
        have_odom_ = true;
      });
    velocity_sub_ = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
      "output/twist", rclcpp::QoS(10), [this](autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg) {
        const rclcpp::Time stamp(msg->header.stamp);
        if (last_velocity_stamp_.nanoseconds() != 0 && stamp <= last_velocity_stamp_) {
          fail("vehicle velocity timestamps did not advance");
          return;
        }
        if (experiment_started_ && last_velocity_stamp_.nanoseconds() != 0) {
          const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - experiment_start_).count();
          const double dt = (stamp - last_velocity_stamp_).seconds();
          const double dv = msg->longitudinal_velocity - latest_speed_;
          if (t > 0.15 && t < 0.85) {
            acceleration_dv_ += dv;
            acceleration_dt_ += dt;
          } else if (t > 2.20 && t < 2.80) {
            braking_dv_ += dv;
            braking_dt_ += dt;
          }
          if (msg->longitudinal_velocity > peak_speed_) peak_speed_ = msg->longitudinal_velocity;
        }
        last_velocity_stamp_ = stamp;
        latest_speed_ = msg->longitudinal_velocity;
        have_velocity_stamp_ = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;
        have_velocity_ = true;
      });
    steering_sub_ = create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
      "output/steering", rclcpp::QoS(10), [this](autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) {
        latest_steering_ = msg->steering_tire_angle;
        if (experiment_started_ && latest_steering_ > peak_steering_) {
          peak_steering_ = latest_steering_;
        }
        have_steering_stamp_ = msg->stamp.sec != 0 || msg->stamp.nanosec != 0;
        have_steering_ = true;
      });
    gear_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearReport>(
      "output/gear_report", rclcpp::QoS(10), [this](autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr msg) {
        latest_gear_ = msg->report;
        have_gear_stamp_ = msg->stamp.sec != 0 || msg->stamp.nanosec != 0;
        have_gear_ = true;
      });
    mode_sub_ = create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
      "output/control_mode_report", rclcpp::QoS(10), [this](autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg) {
        latest_mode_ = msg->mode;
        have_mode_stamp_ = msg->stamp.sec != 0 || msg->stamp.nanosec != 0;
        have_mode_ = true;
      });

    timer_ = create_wall_timer(20ms, std::bind(&VehicleMotionDemo::tick, this));
  }

  bool passed() const { return passed_; }

private:
  void fail(const std::string & reason)
  {
    if (finished_) return;
    finished_ = true;
    passed_ = false;
    RCLCPP_ERROR(get_logger(), "FAIL: %s", reason.c_str());
    rclcpp::shutdown();
  }

  void publish_command(double acceleration, double steering)
  {
    autoware_vehicle_msgs::msg::GearCommand gear;
    gear.stamp = now();
    gear.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    gear_pub_->publish(gear);

    autoware_control_msgs::msg::Control control;
    control.stamp = now();
    control.longitudinal.stamp = control.stamp;
    control.lateral.stamp = control.stamp;
    control.longitudinal.acceleration = acceleration;
    control.longitudinal.is_defined_acceleration = true;
    control.lateral.steering_tire_angle = steering;
    control.lateral.is_defined_steering_tire_rotation_rate = false;
    command_pub_->publish(control);
  }

  bool read_tf(geometry_msgs::msg::TransformStamped & transform)
  {
    try {
      transform = tf_buffer_.lookupTransform(
        "atlas_sim_odom", "atlas_sim_base_link", tf2::TimePointZero, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void publish_marker()
  {
    if (!have_odom_) return;
    visualization_msgs::msg::Marker marker;
    marker.header = last_odom_.header;
    marker.ns = "synthetic_atlas";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = last_odom_.pose.pose;
    const double yaw = tf2::getYaw(marker.pose.orientation);
    marker.pose.position.x += 0.60 * std::cos(yaw);
    marker.pose.position.y += 0.60 * std::sin(yaw);
    marker.pose.position.z += 0.25;
    marker.scale.x = 1.80;
    marker.scale.y = 1.00;
    marker.scale.z = 0.50;
    marker.color.r = 0.95F;
    marker.color.g = 0.28F;
    marker.color.b = 0.05F;
    marker.color.a = 0.90F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.25);
    marker_pub_->publish(marker);
  }

  void tick()
  {
    if (finished_) return;
    const auto now = std::chrono::steady_clock::now();
    if (verification_complete_) {
      publish_command(0.0, 0.0);
      publish_marker();
      if (std::chrono::duration<double>(now - verification_time_).count() >= hold_after_test_sec_) {
        finished_ = true;
        rclcpp::shutdown();
      }
      return;
    }
    if (!experiment_started_) {
      publish_command(0.0, 0.0);
      geometry_msgs::msg::TransformStamped transform;
      if (command_pub_->get_subscription_count() > 0 && have_odom_ && have_velocity_ && have_steering_ &&
          have_gear_ && have_mode_ && read_tf(transform)) {
        initial_odom_ = last_odom_;
        experiment_started_ = true;
        experiment_start_ = now;
        RCLCPP_INFO(get_logger(), "Simulator topics and atlas_sim_odom -> atlas_sim_base_link TF are ready");
      } else if (std::chrono::duration<double>(now - process_start_).count() > timeout_sec_) {
        fail("timed out waiting for simulator command discovery, vehicle status, odometry, and TF");
        return;
      }
      return;
    }

    const double t = std::chrono::duration<double>(now - experiment_start_).count();
    if (t < 0.95) {
      publish_command(0.8, 0.0);
    } else if (t < 2.05) {
      publish_command(0.0, 0.35);
    } else if (t < 3.00) {
      publish_command(-0.8, 0.35);
    } else {
      publish_command(0.0, 0.0);
    }
    publish_marker();

    if (t >= 3.15) {
      geometry_msgs::msg::TransformStamped transform;
      if (!read_tf(transform)) {
        fail("simulator TF was not available at the end of the motion sequence");
        return;
      }
      const auto & pose = last_odom_.pose.pose;
      const double dx = pose.position.x - initial_odom_.pose.pose.position.x;
      const double dy = pose.position.y - initial_odom_.pose.pose.position.y;
      const double distance = std::hypot(dx, dy);
      const double yaw_delta = tf2::getYaw(pose.orientation) - tf2::getYaw(initial_odom_.pose.pose.orientation);
      const double tf_position_error = std::hypot(
        pose.position.x - transform.transform.translation.x,
        pose.position.y - transform.transform.translation.y);
      const double tf_yaw_error = std::abs(
        tf2::getYaw(pose.orientation) - tf2::getYaw(transform.transform.rotation));
      const double measured_acceleration = acceleration_dt_ > 0.0 ? acceleration_dv_ / acceleration_dt_ : 0.0;
      const double measured_braking = braking_dt_ > 0.0 ? braking_dv_ / braking_dt_ : 0.0;

      if (measured_acceleration < 0.5 || measured_acceleration > 1.1)
        return fail("measured acceleration did not track the +0.8 m/s^2 command");
      if (measured_braking > -0.5 || measured_braking < -1.1)
        return fail("measured braking did not track the -0.8 m/s^2 command");
      if (distance < 1.0) return fail("longitudinal displacement assertion failed (< 1.0 m)");
      if (yaw_delta < 0.25 || yaw_delta > 0.70) return fail("heading-change assertion failed (expected 0.25..0.70 rad)");
      if (std::abs(latest_speed_) > 0.15) return fail("vehicle did not sufficiently decelerate at test end");
      if (std::abs(last_odom_.twist.twist.linear.x - latest_speed_) > 0.03)
        return fail("odometry and velocity report speeds differ beyond 0.03 m/s");
      if (std::abs(latest_steering_) > 0.05) return fail("steering report did not return near zero");
      if (std::abs(peak_steering_ - 0.35) > 0.02) return fail("steering report did not track the 0.35 rad command");
      if (latest_gear_ != autoware_vehicle_msgs::msg::GearReport::DRIVE) return fail("forward gear was not reported");
      if (latest_mode_ != autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS) return fail("simulator control mode is not autonomous");
      if (!have_velocity_stamp_ || !have_steering_stamp_ || !have_gear_stamp_ || !have_mode_stamp_)
        return fail("one or more vehicle status messages lacked a timestamp");
      if (tf_position_error > 0.03 || tf_yaw_error > 0.03) return fail("TF and odometry pose differ beyond 0.03 tolerance");
      if (last_odom_stamp_.nanoseconds() == 0) return fail("odometry has no valid timestamp");

      passed_ = true;
      verification_complete_ = true;
      verification_time_ = now;
      RCLCPP_INFO(
        get_logger(), "PASS: acceleration=%.3f m/s^2, braking=%.3f m/s^2, peak_speed=%.3f m/s",
        measured_acceleration, measured_braking, peak_speed_);
      RCLCPP_INFO(
        get_logger(), "PASS: displacement=(%.3f, %.3f) m, distance=%.3f m, heading_change=%.3f rad, "
        "speed=%.3f m/s, gear=DRIVE, TF_error=%.5f m / %.5f rad",
        dx, dy, distance, yaw_delta, latest_speed_, tf_position_error, tf_yaw_error);
      RCLCPP_INFO(
        get_logger(), "PASS: steering_peak=%.3f rad, steering_final=%.3f rad, mode=AUTONOMOUS, "
        "odom_samples=%zu, odom_stamp=%.9f s, status_timestamps=valid",
        peak_steering_, latest_steering_, odom_samples_, last_odom_stamp_.seconds());
      RCLCPP_INFO(
        get_logger(), "PASS: odom_speed=%.3f m/s, TF_stamp=%.9f s, frames=%s -> %s",
        last_odom_.twist.twist.linear.x, rclcpp::Time(transform.header.stamp).seconds(),
        last_odom_.header.frame_id.c_str(), last_odom_.child_frame_id.c_str());
      if (hold_after_test_sec_ <= 0.0) {
        finished_ = true;
        rclcpp::shutdown();
      }
    } else if (std::chrono::duration<double>(now - process_start_).count() > timeout_sec_) {
      fail("motion scenario exceeded its bounded timeout");
    }
  }

  const std::chrono::steady_clock::time_point process_start_{std::chrono::steady_clock::now()};
  double timeout_sec_{12.0};
  double hold_after_test_sec_{0.0};
  bool experiment_started_{false};
  bool verification_complete_{false};
  bool finished_{false};
  bool passed_{false};
  bool have_odom_{false};
  bool have_velocity_{false};
  bool have_steering_{false};
  bool have_gear_{false};
  bool have_mode_{false};
  bool have_velocity_stamp_{false};
  bool have_steering_stamp_{false};
  bool have_gear_stamp_{false};
  bool have_mode_stamp_{false};
  std::chrono::steady_clock::time_point experiment_start_{};
  std::chrono::steady_clock::time_point verification_time_{};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry initial_odom_{};
  nav_msgs::msg::Odometry last_odom_{};
  double latest_speed_{0.0};
  rclcpp::Time last_velocity_stamp_{0, 0, RCL_ROS_TIME};
  double acceleration_dv_{0.0};
  double acceleration_dt_{0.0};
  double braking_dv_{0.0};
  double braking_dt_{0.0};
  double peak_speed_{0.0};
  double latest_steering_{0.0};
  double peak_steering_{0.0};
  size_t odom_samples_{0};
  uint8_t latest_gear_{0};
  uint8_t latest_mode_{0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr command_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleMotionDemo>();
  rclcpp::spin(node);
  return node->passed() ? 0 : 1;
}
