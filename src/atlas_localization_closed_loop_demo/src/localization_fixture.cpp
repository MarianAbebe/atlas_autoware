// Synthetic sensors and fixed test trajectory only. No localization/control algorithm.
#include "stub_pcd_loader.hpp"
#include "test_util.hpp"
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Geometry>
#include <cmath>

using Odom = nav_msgs::msg::Odometry;
using Cloud = sensor_msgs::msg::PointCloud2;
using Twist = geometry_msgs::msg::TwistWithCovarianceStamped;
using Accel = geometry_msgs::msg::AccelWithCovarianceStamped;
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using Gear = autoware_vehicle_msgs::msg::GearCommand;
using Engage = autoware_vehicle_msgs::msg::Engage;
using Mode = autoware_adapi_v1_msgs::msg::OperationModeState;
using namespace std::chrono_literals;

class Fixture : public rclcpp::Node
{
public:
  Fixture() : Node("synthetic_sensor_fixture", "/atlas_loc")
  {
    points_ = create_publisher<Cloud>("sensors/points", rclcpp::SensorDataQoS());
    motion_ = create_publisher<Twist>("sensors/twist", 10);
    accel_ = create_publisher<Accel>("sensors/acceleration", 10);
    trajectory_ = create_publisher<Trajectory>("raw_trajectory", 10);
    sim_init_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("sim/input/initialpose", 10);
    sim_gear_ = create_publisher<Gear>("sim/input/gear_command", 10);
    auto_gear_ = create_publisher<Gear>("auto_gear", 10);
    gate_engage_ = create_publisher<Engage>("gate/input/engage", 10);
    mode_ = create_publisher<Mode>("operation_mode", rclcpp::QoS(1).transient_local());
    truth_ = create_subscription<Odom>("sim/output/odometry", 10, [this](Odom::ConstSharedPtr m) {
      truth_pose_ = m;
      Twist twist;
      twist.header.stamp = m->header.stamp;
      twist.header.frame_id = "base_link";
      // Ideal wheel-speed and gyro sensors at the simulated body origin.
      // Absolute pose is deliberately excluded from this EKF input.
      twist.twist.twist.linear.x = m->twist.twist.linear.x;
      twist.twist.twist.angular.z = m->twist.twist.angular.z;
      twist.twist.covariance[0] = .01 * .01;
      twist.twist.covariance[35] = .005 * .005;
      motion_->publish(twist);
    });
    accel_sub_ = create_subscription<Accel>("sim/output/acceleration", 10, [this](Accel::ConstSharedPtr m) {
      auto sensor = *m;
      sensor.header.frame_id = "base_link";  // identical virtual body axes
      accel_->publish(sensor);
    });
    fault_sub_ = create_subscription<std_msgs::msg::String>("sensor_fault", 10,
      [this](std_msgs::msg::String::ConstSharedPtr m) { drop_scans_ = m->data == "drop_ndt"; });
    tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "synthetic_lidar";
    transform.transform.rotation.w = 1;
    tf_->sendTransform(transform);
    pcl::fromROSMsg(make_default_sensor_pcd(), world_scan_);
    for (auto & p : world_scan_) {p.x += 100; p.y += 100;}
    timer_ = create_wall_timer(20ms, [this] {tick();});
  }
private:
  void tick()
  {
    const auto stamp = now();
    if (!truth_pose_ && sim_init_->get_subscription_count()) {
      auto initial = make_pose(100, 100);
      initial.header.stamp = stamp;
      sim_init_->publish(initial);
    }
    // The pinned SmartPoseBuffer accepts a scan newer than its latest EKF
    // prediction within its existing timestamp tolerance. No artificial wait
    // or fabricated future seed is necessary for this instantaneous sensor.
    if ((stamp-last_reference_).seconds() < .1) return;
    last_reference_ = stamp;
    Trajectory path;
    path.header.frame_id = "map"; path.header.stamp = stamp;
    // Five metres behind the start supply terminal spline support. The 8 m route
    // and bounded spatial speed profile are test inputs, not a new planner.
    for (int i = -100; i <= 160; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint p;
      const double x = i * .05;
      p.pose.position.x = 100+x; p.pose.position.y = 100;
      p.pose.orientation.w = 1;
      p.longitudinal_velocity_mps = x >= 8 ? 0 : 1.;
      path.points.push_back(p);
    }
    trajectory_->publish(path);
    Gear gear; gear.stamp = stamp; gear.command = Gear::DRIVE;
    sim_gear_->publish(gear); auto_gear_->publish(gear);
    Engage engage; engage.stamp = stamp; engage.engage = true; gate_engage_->publish(engage);
    Mode mode; mode.stamp = stamp; mode.mode = Mode::AUTONOMOUS;
    mode.is_autoware_control_enabled = true; mode_->publish(mode);
    if (drop_scans_ || !truth_pose_ || (stamp-rclcpp::Time(truth_pose_->header.stamp)).seconds() > .1) return;
    const auto & pose = truth_pose_->pose.pose;
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    const Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
    pcl::PointCloud<pcl::PointXYZ> local;
    local.reserve(world_scan_.size());
    for (const auto & p : world_scan_) {
      const auto v = q.conjugate() * (Eigen::Vector3d(p.x,p.y,p.z)-t);
      local.push_back(pcl::PointXYZ(v.x(), v.y(), v.z()));
    }
    Cloud scan; pcl::toROSMsg(local, scan);
    scan.header.stamp = truth_pose_->header.stamp;
    scan.header.frame_id = "synthetic_lidar";
    points_->publish(scan);
  }
  bool drop_scans_ = false;
  rclcpp::Time last_reference_{0,0,RCL_ROS_TIME};
  Odom::ConstSharedPtr truth_pose_;
  pcl::PointCloud<pcl::PointXYZ> world_scan_;
  rclcpp::Publisher<Cloud>::SharedPtr points_;
  rclcpp::Publisher<Twist>::SharedPtr motion_;
  rclcpp::Publisher<Accel>::SharedPtr accel_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sim_init_;
  rclcpp::Publisher<Gear>::SharedPtr sim_gear_, auto_gear_;
  rclcpp::Publisher<Engage>::SharedPtr gate_engage_;
  rclcpp::Publisher<Mode>::SharedPtr mode_;
  rclcpp::Subscription<Odom>::SharedPtr truth_;
  rclcpp::Subscription<Accel>::SharedPtr accel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto fixture = std::make_shared<Fixture>();
  // Reuse the unchanged pinned synthetic differential-map fixture.
  auto map = std::make_shared<StubPcdLoader>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(fixture); executor.add_node(map);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
