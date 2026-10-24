#ifndef ATLAS_SENSOR_KIT__POINTCLOUD_CONVERSION_HPP_
#define ATLAS_SENSOR_KIT__POINTCLOUD_CONVERSION_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <cstddef>

namespace atlas_sensor_kit
{
// Conversion is transactional: returns a new cloud, or throws without modifying input.
class ConversionError : public std::invalid_argument
{
public:
  using std::invalid_argument::invalid_argument;
};
struct VelodyneConversion
{
  sensor_msgs::msg::PointCloud2 cloud;
  size_t discarded_nonfinite_xyz = 0;
  bool reordered = false;
};
// Explicit contract: FLOAT32 time is seconds relative to the unchanged header.
// Skips nonfinite XYZ, stable-sorts retained points by time, rounds to UINT32 ns.
VelodyneConversion velodyne_relative_seconds_to_extended(
  const sensor_msgs::msg::PointCloud2 & input);
sensor_msgs::msg::PointCloud2 livox_to_extended(const sensor_msgs::msg::PointCloud2 & input);
sensor_msgs::msg::PointCloud2 extended_to_compact(const sensor_msgs::msg::PointCloud2 & input);
}  // namespace atlas_sensor_kit
#endif
