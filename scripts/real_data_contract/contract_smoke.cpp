// Offline contract probe of actual serialized messages; no TF or motion invented.
#include <atlas_sensor_kit/pointcloud_conversion.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  std::ifstream f(argv[1], std::ios::binary);
  if (!f) return 2;
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), {});
  if (bytes.empty() || bytes.size() > 4000000) return 2;
  rclcpp::SerializedMessage serialized(bytes.size());
  auto &raw = serialized.get_rcl_serialized_message();
  std::copy(bytes.begin(), bytes.end(), raw.buffer);
  raw.buffer_length = bytes.size();
  sensor_msgs::msg::PointCloud2 cloud;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
  serializer.deserialize_message(&serialized, &cloud);
  std::cout << "Recorded frame=" << cloud.header.frame_id << " points="
            << uint64_t(cloud.width)*cloud.height << " stride=" << cloud.point_step << '\n';
  int rejected=0;
  for (auto convert : {atlas_sensor_kit::livox_to_extended, atlas_sensor_kit::extended_to_compact}) {
    try { auto out=convert(cloud); std::cout << "Accepted: " << out.data.size() << " bytes\n"; }
    catch (const atlas_sensor_kit::ConversionError &e) {
      std::cout << "Rejected: " << e.what() << '\n'; ++rejected;
    }
  }
  std::cout << "Expected incompatible-input rejections=" << rejected << "/2\n";
  return rejected == 2 ? 0 : 1;
}
