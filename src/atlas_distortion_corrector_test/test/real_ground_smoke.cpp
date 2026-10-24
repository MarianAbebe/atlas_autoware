// Bounded real-message transport probe, using registered components and no TF/motion.
#include <class_loader/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <thread>
using Cloud = sensor_msgs::msg::PointCloud2;
using namespace std::chrono_literals;
Cloud load(const char * path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open input");
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(f)), {});
  if (bytes.empty() || bytes.size() > 4000000) throw std::runtime_error("invalid CDR size");
  rclcpp::SerializedMessage serialized(bytes.size());
  auto & raw = serialized.get_rcl_serialized_message();
  std::copy(bytes.begin(), bytes.end(), raw.buffer); raw.buffer_length = bytes.size();
  Cloud cloud;
  rclcpp::Serialization<Cloud>().deserialize_message(&serialized, &cloud);
  return cloud;
}
void save(const char * path, const Cloud & cloud) {
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<Cloud>().serialize_message(&cloud, &serialized);
  const auto & raw = serialized.get_rcl_serialized_message();
  std::ofstream f(path, std::ios::binary);
  f.write(reinterpret_cast<const char *>(raw.buffer), raw.buffer_length);
  if (!f) throw std::runtime_error("cannot write output");
}
int main(int argc, char ** argv) {
 try {
  if (argc != 3) throw std::runtime_error("usage: real_ground_smoke compact.cdr output.cdr");
  const auto input = load(argv[1]);
  rclcpp::init(0, nullptr);
  auto context = rclcpp::contexts::get_global_default_context();
  class_loader::ClassLoader loader(ATLAS_GROUND_LIBRARY);
  const std::string name =
    "rclcpp_components::NodeFactoryTemplate<autoware::ground_segmentation::RANSACGroundFilterComponent>";
  auto factory = loader.createInstance<rclcpp_components::NodeFactory>(name);
  rclcpp::NodeOptions options;
  // This is the original sensor frame itself, not an identity vehicle extrinsic.
  // Keep all plane-fitting thresholds unchanged from the pinned YAML.
  options.context(context).use_intra_process_comms(false).arguments({"--ros-args",
    "--params-file", ATLAS_GROUND_PARAMS,
    "-r", "input:=/atlas_real_ground/input", "-r", "output:=/atlas_real_ground/output"});
  options.parameter_overrides({rclcpp::Parameter("base_frame", input.header.frame_id),
    rclcpp::Parameter("input_frame", input.header.frame_id),
    rclcpp::Parameter("output_frame", input.header.frame_id)});
  auto component = factory->create_node_instance(options);
  auto probe = std::make_shared<rclcpp::Node>("atlas_real_ground_probe",
    rclcpp::NodeOptions().context(context).use_intra_process_comms(false));
  Cloud::ConstSharedPtr received;
  auto sub = probe->create_subscription<Cloud>("/atlas_real_ground/output",rclcpp::SensorDataQoS(),
    [&received](Cloud::ConstSharedPtr cloud) {received=cloud;});
  auto pub = probe->create_publisher<Cloud>("/atlas_real_ground/input",rclcpp::SensorDataQoS());
  rclcpp::ExecutorOptions executor_options; executor_options.context=context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(component.get_node_base_interface()); executor.add_node(probe);
  const auto deadline=std::chrono::steady_clock::now()+10s;
  auto next_publish=std::chrono::steady_clock::now();
  while (!received && std::chrono::steady_clock::now()<deadline) {
    if (pub->get_subscription_count() && std::chrono::steady_clock::now()>=next_publish) {
      pub->publish(input); next_publish=std::chrono::steady_clock::now()+500ms;
    }
    executor.spin_some(10ms); std::this_thread::sleep_for(10ms);
  }
  if (!received) throw std::runtime_error("RANSAC output timeout");
  save(argv[2], *received);
  std::cout << "Pinned RANSAC in native sensor frame: input=" << input.width
            << " output=" << received->width << "; sensor-z plane filter only, no ground truth\n";
  context->shutdown("real plane-filter probe complete");
  return 0;
 } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
