#ifndef ATLAS_NAV2_DEMO__ACKERMANN_CONTROLLER_HPP_
#define ATLAS_NAV2_DEMO__ACKERMANN_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace atlas_nav2_demo
{

class AckermannController final : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  using CollisionChecker = nav2_costmap_2d::FootprintCollisionChecker<
    nav2_costmap_2d::Costmap2D *>;

  bool footprintClear(double x, double y, double yaw,
                      const std::vector<geometry_msgs::msg::Point> & footprint);
  bool rolloutClear(double x, double y, double yaw, double speed,
                    double steering, double target_speed,
                    double target_steering, double duration,
                    double step, const std::vector<geometry_msgs::msg::Point> & footprint,
                    double & minimum_cost, double command_delay = 0.0);
  geometry_msgs::msg::TwistStamped zeroCommand(const std_msgs::msg::Header & header) const;
  void onLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  bool obstacleInputFresh() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  CollisionChecker collision_checker_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    compute_trace_publisher_;
  mutable std::mutex scan_mutex_;
  std::chrono::steady_clock::time_point last_scan_receive_{};
  int64_t last_scan_source_ns_{0};
  bool have_valid_scan_{false};
  nav_msgs::msg::Path plan_;
  std::string plugin_name_;
  std::string global_frame_;
  std::string base_frame_;
  double wheelbase_{1.2};
  double steering_limit_{0.5};
  double steering_rate_limit_{0.6};
  double acceleration_limit_{0.8};
  double deceleration_limit_{1.2};
  double max_speed_{0.35};
  double lookahead_{1.0};
  double minimum_lookahead_{0.5};
  double lookahead_step_{0.1};
  double controller_period_{0.1};
  double reaction_time_{0.37};
  double stopping_buffer_{0.05};
  double collision_step_{0.05};
  double obstacle_scan_timeout_{0.30};
  double goal_position_tolerance_{0.10};
  double goal_heading_tolerance_{0.12};
  double cusp_stop_tolerance_{0.04};
  double speed_limit_{0.35};
  double last_steering_{0.0};
  double last_command_speed_{0.0};
  std::size_t active_segment_{0};
  uint64_t compute_trace_sequence_{0};
  int active_direction_{0};
  bool active_{false};
  double last_collision_x_{0.0};
  double last_collision_y_{0.0};
  double last_collision_yaw_{0.0};
  double last_collision_cost_{0.0};
};

}  // namespace atlas_nav2_demo

#endif  // ATLAS_NAV2_DEMO__ACKERMANN_CONTROLLER_HPP_
