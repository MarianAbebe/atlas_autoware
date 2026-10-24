#include "atlas_sensor_kit/pointcloud_conversion.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>
#include <utility>

namespace atlas_sensor_kit
{
class ConversionComponent : public rclcpp::Node
{
public:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Convert = Cloud (*)(const Cloud &);
  ConversionComponent(const char * name, const rclcpp::NodeOptions & options, Convert convert)
  : Node(name, options)
  {
    publisher_ = create_publisher<Cloud>("~/output/pointcloud", rclcpp::SensorDataQoS());
    subscriber_ = create_subscription<Cloud>("~/input/pointcloud", rclcpp::SensorDataQoS(),
      [this, convert](Cloud::ConstSharedPtr input) {
        try {
          publisher_->publish(std::make_unique<Cloud>(convert(*input)));
        } catch (const ConversionError & error) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
            "Rejected point cloud: %s", error.what());
        }
      });
  }
private:
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<Cloud>::SharedPtr subscriber_;
};
class LivoxToExtendedComponent : public ConversionComponent
{
public:
  explicit LivoxToExtendedComponent(const rclcpp::NodeOptions & options)
  : ConversionComponent("livox_to_extended", options, livox_to_extended) {}
};
class ExtendedToCompactComponent : public ConversionComponent
{
public:
  explicit ExtendedToCompactComponent(const rclcpp::NodeOptions & options)
  : ConversionComponent("extended_to_compact", options, extended_to_compact) {}
};
class VelodyneToExtendedComponent : public rclcpp::Node
{
public:
  explicit VelodyneToExtendedComponent(const rclcpp::NodeOptions & options)
  : Node("velodyne_to_extended", options)
  {
    if (!declare_parameter<bool>("time_is_relative_seconds", false)) {
      throw std::invalid_argument("Set time_is_relative_seconds=true only for header-relative seconds");
    }
    publisher_ = create_publisher<Cloud>("~/output/pointcloud", rclcpp::SensorDataQoS());
    subscriber_ = create_subscription<Cloud>("~/input/pointcloud", rclcpp::SensorDataQoS(),
      [this](Cloud::ConstSharedPtr input) {
        try {
          auto result = velodyne_relative_seconds_to_extended(*input);
          RCLCPP_INFO(get_logger(),
            "Converted %u points; discarded nonfinite XYZ: %zu; reordered: %s",
            result.cloud.width, result.discarded_nonfinite_xyz, result.reordered ? "yes" : "no");
          publisher_->publish(std::move(result.cloud));
        } catch (const ConversionError & error) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected point cloud: %s", error.what());
        }
      });
  }
private:
  using Cloud = sensor_msgs::msg::PointCloud2;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Subscription<Cloud>::SharedPtr subscriber_;
};
}  // namespace atlas_sensor_kit
RCLCPP_COMPONENTS_REGISTER_NODE(atlas_sensor_kit::LivoxToExtendedComponent)
RCLCPP_COMPONENTS_REGISTER_NODE(atlas_sensor_kit::ExtendedToCompactComponent)

RCLCPP_COMPONENTS_REGISTER_NODE(atlas_sensor_kit::VelodyneToExtendedComponent)
