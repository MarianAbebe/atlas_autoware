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
  if (argc != 4) throw std::runtime_error("usage: real_cloud_smoke input.cdr extended.cdr compact.cdr");
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  // Load the actual registered factories; do not call conversion functions directly.
  class_loader::ClassLoader loader(ATLAS_COMPONENT_LIBRARY);
  const auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  const std::string first_class =
    "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::VelodyneToExtendedComponent>";
  const std::string second_class =
    "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::ExtendedToCompactComponent>";
  if (std::find(classes.begin(), classes.end(), first_class) == classes.end()) throw std::runtime_error("missing Velodyne factory");
  if (std::find(classes.begin(), classes.end(), second_class) == classes.end()) throw std::runtime_error("missing compact factory");
  auto first_factory = loader.createInstance<rclcpp_components::NodeFactory>(first_class);
  auto second_factory = loader.createInstance<rclcpp_components::NodeFactory>(second_class);
  rclcpp::NodeOptions first_options;
  first_options.parameter_overrides({rclcpp::Parameter("time_is_relative_seconds", true)});
  first_options.context(context).use_intra_process_comms(false).arguments({"--ros-args",
    "-r", "~/input/pointcloud:=/atlas_adapter_test/input",
    "-r", "~/output/pointcloud:=/atlas_adapter_test/extended"});
  rclcpp::NodeOptions second_options;
  second_options.context(context).use_intra_process_comms(false).arguments({"--ros-args",
    "-r", "~/input/pointcloud:=/atlas_adapter_test/extended",
    "-r", "~/output/pointcloud:=/atlas_adapter_test/compact"});
  auto first = first_factory->create_node_instance(first_options);
  auto second = second_factory->create_node_instance(second_options);
  auto probe = std::make_shared<rclcpp::Node>("atlas_adapter_test_probe",
    rclcpp::NodeOptions().context(context).use_intra_process_comms(false));
  Cloud::ConstSharedPtr extended_received, compact_received;
  auto extended_sub = probe->create_subscription<Cloud>("/atlas_adapter_test/extended",
    rclcpp::SensorDataQoS(), [&extended_received](Cloud::ConstSharedPtr c) {extended_received = c;});
  auto compact_sub = probe->create_subscription<Cloud>("/atlas_adapter_test/compact",
    rclcpp::SensorDataQoS(), [&compact_received](Cloud::ConstSharedPtr c) {compact_received = c;});
  auto publisher = probe->create_publisher<Cloud>("/atlas_adapter_test/input", rclcpp::SensorDataQoS());
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(first.get_node_base_interface());
  executor.add_node(second.get_node_base_interface());
  executor.add_node(probe);
  auto ready = [&]() {
      return publisher->get_subscription_count() >= 1 &&
        probe->count_subscribers("/atlas_adapter_test/extended") >= 2 &&
        probe->count_publishers("/atlas_adapter_test/extended") >= 1 &&
        probe->count_publishers("/atlas_adapter_test/compact") >= 1;
    };
  const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while (!ready() && std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some(10ms); std::this_thread::sleep_for(10ms);
  }
  if (!ready()) throw std::runtime_error("ROS graph discovery timed out");
  const auto input = load(argv[1]);
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  auto next_publish = std::chrono::steady_clock::now();
  while ((!extended_received || !compact_received) && std::chrono::steady_clock::now() < deadline) {
    if (std::chrono::steady_clock::now() >= next_publish) {
      publisher->publish(input); next_publish = std::chrono::steady_clock::now() + 100ms;
    }
    executor.spin_some(10ms); std::this_thread::sleep_for(10ms);
  }
  if (!extended_received || !compact_received) throw std::runtime_error("output timed out");
  save(argv[2], *extended_received); save(argv[3], *compact_received);
  std::cout << "Recorded input slots=" << uint64_t(input.width)*input.height
            << " output=" << extended_received->width << " frame=" << extended_received->header.frame_id << '\n';
  context->shutdown("real-data probe complete");
  return 0;
 } catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
}
