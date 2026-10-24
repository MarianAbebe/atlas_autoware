// Exercise only the safe pinned algorithm boundary: valid layout + empty motion queue.
#include <autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
int main(int argc, char **argv)
{
  if (argc != 2) return 2;
  std::ifstream f(argv[1], std::ios::binary);
  if (!f) return 2;
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), {});
  if (bytes.empty() || bytes.size() > 4000000) return 2;
  rclcpp::SerializedMessage serialized(bytes.size());
  auto & raw = serialized.get_rcl_serialized_message();
  std::copy(bytes.begin(), bytes.end(), raw.buffer); raw.buffer_length = bytes.size();
  sensor_msgs::msg::PointCloud2 cloud;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2>().deserialize_message(&serialized, &cloud);
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("atlas_real_deskew_boundary");
  autoware::pointcloud_preprocessor::DistortionCorrector2D corrector(*node);
  if (!corrector.is_pointcloud_valid(cloud) || !corrector.get_twist_queue().empty()) return 1;
  const auto before = cloud;
  // Do not call set_pointcloud_transform: missing physical calibration is a blocker.
  // Pinned implementation returns before using transforms when its twist queue is empty.
  corrector.undistort_pointcloud(false, std::nullopt, cloud);
  if (cloud != before) return 1;
  std::cout << "Pinned deskewer layout accepted; twist queue empty; cloud unchanged. "
               "Motion compensation NOT validated; TF boundary not crossed.\n";
  rclcpp::shutdown();
  return 0;
}
