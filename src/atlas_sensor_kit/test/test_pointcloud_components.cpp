#include <class_loader/class_loader.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <thread>

namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
using namespace std::chrono_literals;
template<typename T> void put(Cloud & c, size_t offset, T value)
{
  std::memcpy(c.data.data() + offset, &value, sizeof(T));
}
template<typename T> T get(const Cloud & c, size_t offset)
{
  T value;
  std::memcpy(&value, c.data.data() + offset, sizeof(T));
  return value;
}
Cloud synthetic_livox()
{
  Cloud c;
  c.header.frame_id = "synthetic_sensor";
  c.header.stamp.sec = 1700000000;
  c.header.stamp.nanosec = 256;  // Exact FLOAT64 epoch representation, deterministic.
  c.height = 1; c.width = 3; c.point_step = 26; c.row_step = 78; c.is_dense = true;
  const std::array<const char *, 7> names{{"x", "y", "z", "intensity", "tag", "line", "timestamp"}};
  const std::array<uint32_t, 7> offsets{{0, 4, 8, 12, 16, 17, 18}};
  for (size_t i = 0; i < names.size(); ++i) {
    Field f;
    f.name = names[i]; f.offset = offsets[i]; f.count = 1;
    f.datatype = i < 4 ? Field::FLOAT32 : (i < 6 ? Field::UINT8 : Field::FLOAT64);
    c.fields.push_back(f);
  }
  c.data.resize(c.row_step);
  const std::array<std::array<float, 4>, 3> values{{
    {{1, 2, 3, 12.5F}}, {{-4, 5, -6, 300}}, {{7, -8, 9, -2}}}};
  for (size_t i = 0; i < values.size(); ++i) {
    for (size_t j = 0; j < 4; ++j) put<float>(c, i * 26 + j * 4, values[i][j]);
    c.data[i * 26 + 16] = 255; c.data[i * 26 + 17] = 3;
    put<double>(c, i * 26 + 18, static_cast<double>(1700000000000000256ULL + i * 1024));
  }
  return c;
}

void check_layout(const Cloud & c, bool extended)
{
  const std::array<const char *, 10> names{{"x", "y", "z", "intensity", "return_type",
    "channel", "azimuth", "elevation", "distance", "time_stamp"}};
  const std::array<uint32_t, 10> offsets{{0, 4, 8, 12, 13, 14, 16, 20, 24, 28}};
  const std::array<uint8_t, 10> types{{Field::FLOAT32, Field::FLOAT32, Field::FLOAT32,
    Field::UINT8, Field::UINT8, Field::UINT16, Field::FLOAT32, Field::FLOAT32,
    Field::FLOAT32, Field::UINT32}};
  const size_t count = extended ? 10 : 6;
  ASSERT_EQ(c.fields.size(), count);
  for (size_t i = 0; i < count; ++i) {
    EXPECT_EQ(c.fields[i].name, names[i]); EXPECT_EQ(c.fields[i].offset, offsets[i]);
    EXPECT_EQ(c.fields[i].datatype, types[i]); EXPECT_EQ(c.fields[i].count, 1U);
  }
  EXPECT_EQ(c.height, 1U); EXPECT_EQ(c.width, 3U);
  EXPECT_EQ(c.point_step, extended ? 32U : 16U);
  EXPECT_EQ(c.row_step, c.width * c.point_step);
  EXPECT_EQ(c.data.size(), c.row_step);
  EXPECT_FALSE(c.is_bigendian); EXPECT_TRUE(c.is_dense);
}

class ComponentIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
  }
  void TearDown() override
  {
    if (context_ && context_->is_valid()) context_->shutdown("test complete");
  }
  rclcpp::Context::SharedPtr context_;
};

TEST_F(ComponentIntegration, RegisteredComponentsConvertThroughRosTopics)
{
  // Load the actual registered factories; do not call conversion functions directly.
  class_loader::ClassLoader loader(ATLAS_COMPONENT_LIBRARY);
  const auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  const std::string first_class =
    "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::LivoxToExtendedComponent>";
  const std::string second_class =
    "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::ExtendedToCompactComponent>";
  ASSERT_NE(std::find(classes.begin(), classes.end(), first_class), classes.end());
  ASSERT_NE(std::find(classes.begin(), classes.end(), second_class), classes.end());
  auto first_factory = loader.createInstance<rclcpp_components::NodeFactory>(first_class);
  auto second_factory = loader.createInstance<rclcpp_components::NodeFactory>(second_class);
  rclcpp::NodeOptions first_options;
  first_options.context(context_).use_intra_process_comms(false).arguments({"--ros-args",
    "-r", "~/input/pointcloud:=/atlas_adapter_test/input",
    "-r", "~/output/pointcloud:=/atlas_adapter_test/extended"});
  rclcpp::NodeOptions second_options;
  second_options.context(context_).use_intra_process_comms(false).arguments({"--ros-args",
    "-r", "~/input/pointcloud:=/atlas_adapter_test/extended",
    "-r", "~/output/pointcloud:=/atlas_adapter_test/compact"});
  auto first = first_factory->create_node_instance(first_options);
  auto second = second_factory->create_node_instance(second_options);
  auto probe = std::make_shared<rclcpp::Node>("atlas_adapter_test_probe",
    rclcpp::NodeOptions().context(context_).use_intra_process_comms(false));
  Cloud::ConstSharedPtr extended_received, compact_received;
  auto extended_sub = probe->create_subscription<Cloud>("/atlas_adapter_test/extended",
    rclcpp::SensorDataQoS(), [&extended_received](Cloud::ConstSharedPtr c) {extended_received = c;});
  auto compact_sub = probe->create_subscription<Cloud>("/atlas_adapter_test/compact",
    rclcpp::SensorDataQoS(), [&compact_received](Cloud::ConstSharedPtr c) {compact_received = c;});
  auto publisher = probe->create_publisher<Cloud>("/atlas_adapter_test/input", rclcpp::SensorDataQoS());
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
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
  ASSERT_TRUE(ready()) << "ROS graph discovery timed out";
  const auto input = synthetic_livox();
  const auto input_before = input.data;
  const auto delivery_deadline = std::chrono::steady_clock::now() + 5s;
  auto next_publish = std::chrono::steady_clock::now();
  while ((!extended_received || !compact_received) &&
    std::chrono::steady_clock::now() < delivery_deadline) {
    if (std::chrono::steady_clock::now() >= next_publish) {
      publisher->publish(input); next_publish = std::chrono::steady_clock::now() + 100ms;
    }
    executor.spin_some(10ms); std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(extended_received) << "Extended output timed out";
  ASSERT_TRUE(compact_received) << "Compact output timed out";
  check_layout(*extended_received, true); check_layout(*compact_received, false);
  ASSERT_EQ(extended_received->data.size(), 96U);
  ASSERT_EQ(compact_received->data.size(), 48U);
  EXPECT_EQ(extended_received->header, input.header);
  EXPECT_EQ(compact_received->header, input.header);
  EXPECT_EQ(input.data, input_before);
  const std::array<uint8_t, 3> intensity{{13, 255, 0}};
  constexpr double pi = 3.14159265358979323846;
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      EXPECT_FLOAT_EQ(get<float>(*extended_received, i * 32 + j * 4), get<float>(input, i * 26 + j * 4));
      EXPECT_FLOAT_EQ(get<float>(*compact_received, i * 16 + j * 4), get<float>(input, i * 26 + j * 4));
    }
    EXPECT_EQ(extended_received->data[i * 32 + 12], intensity[i]);
    EXPECT_EQ(compact_received->data[i * 16 + 12], intensity[i]);
    EXPECT_EQ(extended_received->data[i * 32 + 13], 0U);
    EXPECT_EQ(compact_received->data[i * 16 + 13], 0U);
    EXPECT_EQ(get<uint16_t>(*extended_received, i * 32 + 14), 0U);
    EXPECT_EQ(get<uint16_t>(*compact_received, i * 16 + 14), 0U);
    EXPECT_EQ(get<uint32_t>(*extended_received, i * 32 + 28), i * 1024U);
    const double x = get<float>(input, i * 26), y = get<float>(input, i * 26 + 4),
      z = get<float>(input, i * 26 + 8);
    double azimuth = std::atan2(y, x); if (azimuth < 0) azimuth += 2 * pi;
    EXPECT_NEAR(get<float>(*extended_received, i * 32 + 16), azimuth, 1e-6);
    EXPECT_NEAR(get<float>(*extended_received, i * 32 + 20), std::atan2(z, std::hypot(x, y)), 1e-6);
    EXPECT_NEAR(get<float>(*extended_received, i * 32 + 24), std::hypot(std::hypot(x, y), z), 1e-5);
  }
  // Compact output has no per-point timestamp: only the unchanged cloud timestamp remains.
  EXPECT_EQ(std::count_if(compact_received->fields.begin(), compact_received->fields.end(),
    [](const Field & f) {return f.name == "timestamp" || f.name == "time_stamp";}), 0);
}
}  // namespace
