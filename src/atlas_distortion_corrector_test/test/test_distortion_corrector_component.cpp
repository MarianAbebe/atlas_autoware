#include <class_loader/class_loader.hpp>
#include <gtest/gtest.h>
#include <managed_transform_buffer/managed_transform_buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <optional>
#include <thread>

namespace
{
using namespace std::chrono_literals;
using Cloud = sensor_msgs::msg::PointCloud2;
using Field = sensor_msgs::msg::PointField;
using Twist = geometry_msgs::msg::TwistWithCovarianceStamped;
constexpr char base_frame[] = "atlas_deskew_test_base";
constexpr char cloud_frame[] = "atlas_deskew_test_lidar";
template<typename T> void put(Cloud & c, size_t at, T value)
{ std::memcpy(c.data.data() + at, &value, sizeof(T)); }
template<typename T> T get(const Cloud & c, size_t at)
{ T value; std::memcpy(&value, c.data.data() + at, sizeof(T)); return value; }
Cloud cloud()
{
  Cloud c;
  c.header.frame_id = cloud_frame; c.header.stamp.sec = 100;
  c.height = 1; c.width = 2; c.point_step = 32; c.row_step = 64;
  c.is_dense = true; c.is_bigendian = false;
  const std::array<const char *, 10> names{{"x", "y", "z", "intensity", "return_type",
    "channel", "azimuth", "elevation", "distance", "time_stamp"}};
  const std::array<uint32_t, 10> offsets{{0, 4, 8, 12, 13, 14, 16, 20, 24, 28}};
  const std::array<uint8_t, 10> types{{Field::FLOAT32, Field::FLOAT32, Field::FLOAT32,
    Field::UINT8, Field::UINT8, Field::UINT16, Field::FLOAT32, Field::FLOAT32,
    Field::FLOAT32, Field::UINT32}};
  for (size_t i = 0; i < names.size(); ++i) {
    Field f; f.name = names[i]; f.offset = offsets[i]; f.datatype = types[i]; f.count = 1;
    c.fields.push_back(f);
  }
  c.data.resize(64, 0);
  for (size_t i = 0; i < 2; ++i) {
    const size_t at = i * 32;
    put<float>(c, at, 1.0F);
    put<uint8_t>(c, at + 12, static_cast<uint8_t>(21 + i));
    put<uint8_t>(c, at + 13, static_cast<uint8_t>(1 + i));
    put<uint16_t>(c, at + 14, static_cast<uint16_t>(7 + i));
    put<float>(c, at + 24, 1.0F);
    put<uint32_t>(c, at + 28, i == 0 ? 0U : 50000000U);
  }
  return c;
}
Cloud livox_cloud()
{
  Cloud c;
  c.header.frame_id = cloud_frame; c.header.stamp.sec = 100;
  c.height = 1; c.width = 2; c.point_step = 26; c.row_step = 52;
  c.is_dense = true; c.is_bigendian = false;
  const std::array<const char *, 7> names{{"x", "y", "z", "intensity", "tag", "line", "timestamp"}};
  const std::array<uint32_t, 7> offsets{{0, 4, 8, 12, 16, 17, 18}};
  for (size_t i = 0; i < names.size(); ++i) {
    Field f; f.name = names[i]; f.offset = offsets[i]; f.count = 1;
    f.datatype = i < 4 ? Field::FLOAT32 : (i < 6 ? Field::UINT8 : Field::FLOAT64);
    c.fields.push_back(f);
  }
  c.data.resize(52, 0);
  for (size_t i = 0; i < 2; ++i) {
    const size_t at = i * 26;
    put<float>(c, at, 1.0F);
    put<float>(c, at + 12, i == 0 ? 21.5F : 87.25F);  // Expected uint8: 22, 87.
    put<uint8_t>(c, at + 16, static_cast<uint8_t>(32 + i));
    put<uint8_t>(c, at + 17, static_cast<uint8_t>(4 + i));
    put<double>(c, at + 18, i == 0 ? 100000000000.0 : 100050000000.0);
  }
  return c;
}
Cloud expected_extended_cloud()
{
  auto c = cloud();
  for (size_t i = 0; i < 2; ++i) {
    put<uint8_t>(c, i * 32 + 12, i == 0 ? 22 : 87);
    put<uint8_t>(c, i * 32 + 13, 0);  // Atlas provisional metadata, not Livox tag.
    put<uint16_t>(c, i * 32 + 14, 0); // Atlas provisional metadata, not Livox line.
  }
  return c;
}
void verify_compact(const Cloud & output, const Cloud & input, float second_x)
{
  const auto extended = expected_extended_cloud();
  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(output.width, 2U); EXPECT_EQ(output.height, 1U);
  EXPECT_EQ(output.point_step, 16U); EXPECT_EQ(output.row_step, 32U);
  EXPECT_FALSE(output.is_bigendian); EXPECT_TRUE(output.is_dense);
  ASSERT_EQ(output.fields.size(), 6U);
  for (size_t i = 0; i < 6; ++i) EXPECT_EQ(output.fields[i], extended.fields[i]);
  ASSERT_EQ(output.data.size(), 32U);
  for (size_t i = 0; i < 2; ++i) {
    const size_t at = i * 16;
    EXPECT_NEAR(get<float>(output, at), i == 0 ? 1.0F : second_x, 1e-5F);
    EXPECT_NEAR(get<float>(output, at + 4), 0.0F, 1e-5F);
    EXPECT_NEAR(get<float>(output, at + 8), 0.0F, 1e-5F);
    EXPECT_EQ(get<uint8_t>(output, at + 12), i == 0 ? 22U : 87U);
    EXPECT_EQ(get<uint8_t>(output, at + 13), 0U);
    EXPECT_EQ(get<uint16_t>(output, at + 14), 0U);
  }
}
void verify(const Cloud & output, const Cloud & input, float second_x)
{
  EXPECT_EQ(output.header, input.header);
  EXPECT_EQ(output.height, 1U); EXPECT_EQ(output.width, 2U);
  EXPECT_EQ(output.point_step, 32U); EXPECT_EQ(output.row_step, 64U);
  EXPECT_EQ(output.fields, input.fields);
  EXPECT_EQ(output.is_bigendian, input.is_bigendian);
  EXPECT_EQ(output.is_dense, input.is_dense);
  ASSERT_EQ(output.data.size(), 64U);
  for (size_t i = 0; i < 2; ++i) {
    const size_t at = i * 32;
    EXPECT_NEAR(get<float>(output, at), i == 0 ? 1.0F : second_x, 1e-5F);
    EXPECT_NEAR(get<float>(output, at + 4), 0.0F, 1e-5F);
    EXPECT_NEAR(get<float>(output, at + 8), 0.0F, 1e-5F);
    EXPECT_EQ(get<uint32_t>(output, at + 28), i == 0 ? 0U : 50000000U);
    EXPECT_EQ(get<uint8_t>(output, at + 12), get<uint8_t>(input, at + 12));
    EXPECT_EQ(get<uint8_t>(output, at + 13), get<uint8_t>(input, at + 13));
    EXPECT_EQ(get<uint16_t>(output, at + 14), get<uint16_t>(input, at + 14));
    for (size_t j = 12; j < 32; ++j) {
      EXPECT_EQ(output.data[at + j], input.data[at + j]) << "point " << i << " byte " << j;
    }
  }
}
class DistortionCorrector : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
  void run(const std::string & scenario, bool chain = false)
  {
    class_loader::ClassLoader loader(ATLAS_COMPONENT_LIBRARY);
    const std::string factory_name =
      "rclcpp_components::NodeFactoryTemplate<autoware::pointcloud_preprocessor::DistortionCorrectorComponent>";
    const auto factories = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
    ASSERT_NE(std::find(factories.begin(), factories.end(), factory_name), factories.end());
    auto factory = loader.createInstance<rclcpp_components::NodeFactory>(factory_name);
    std::unique_ptr<class_loader::ClassLoader> converter_loader;
    std::shared_ptr<rclcpp_components::NodeFactory> first_factory, last_factory;
    std::optional<rclcpp_components::NodeInstanceWrapper> first, last;
    if (chain) {
      converter_loader = std::make_unique<class_loader::ClassLoader>(ATLAS_CONVERTER_LIBRARY);
      const auto classes = converter_loader->getAvailableClasses<rclcpp_components::NodeFactory>();
      const std::string first_name =
        "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::LivoxToExtendedComponent>";
      const std::string last_name =
        "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::ExtendedToCompactComponent>";
      ASSERT_NE(std::find(classes.begin(), classes.end(), first_name), classes.end());
      ASSERT_NE(std::find(classes.begin(), classes.end(), last_name), classes.end());
      first_factory = converter_loader->createInstance<rclcpp_components::NodeFactory>(first_name);
      last_factory = converter_loader->createInstance<rclcpp_components::NodeFactory>(last_name);
      rclcpp::NodeOptions first_options, last_options;
      first_options.use_intra_process_comms(false).arguments({"--ros-args", "-r",
        "__ns:=/atlas_deskew_test", "-r", "~/input/pointcloud:=/atlas_deskew_test/input",
        "-r", "~/output/pointcloud:=/atlas_deskew_test/extended"});
      last_options.use_intra_process_comms(false).arguments({"--ros-args", "-r",
        "__ns:=/atlas_deskew_test", "-r", "~/input/pointcloud:=/atlas_deskew_test/rectified",
        "-r", "~/output/pointcloud:=/atlas_deskew_test/output"});
      first.emplace(first_factory->create_node_instance(first_options));
      last.emplace(last_factory->create_node_instance(last_options));
    }
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false).arguments({"--ros-args", "-r",
      "__ns:=/atlas_deskew_test", "-r",
      chain ? "~/input/pointcloud:=/atlas_deskew_test/extended" : "~/input/pointcloud:=/atlas_deskew_test/input",
      "-r", "~/input/twist:=/atlas_deskew_test/twist", "-r",
      chain ? "~/output/pointcloud:=/atlas_deskew_test/rectified" : "~/output/pointcloud:=/atlas_deskew_test/output"});
    options.parameter_overrides({rclcpp::Parameter("base_frame", base_frame),
      rclcpp::Parameter("use_imu", false),
      rclcpp::Parameter("use_3d_distortion_correction", false),
      rclcpp::Parameter("update_azimuth_and_distance", false),
      rclcpp::Parameter("processing_time_threshold_sec", 0.01),
      rclcpp::Parameter("timestamp_mismatch_fraction_threshold", 0.01)});
    auto component = factory->create_node_instance(options);
    auto probe = std::make_shared<rclcpp::Node>("probe", "/atlas_deskew_test",
      rclcpp::NodeOptions().use_intra_process_comms(false));
    Cloud::ConstSharedPtr received, extended_received, rectified_received;
    rclcpp::Subscription<Cloud>::SharedPtr extended_sub, rectified_sub;
    if (chain) {
      extended_sub = probe->create_subscription<Cloud>("extended", rclcpp::SensorDataQoS(),
        [&extended_received](Cloud::ConstSharedPtr msg) {extended_received = msg;});
      rectified_sub = probe->create_subscription<Cloud>("rectified", rclcpp::SensorDataQoS(),
        [&rectified_received](Cloud::ConstSharedPtr msg) {rectified_received = msg;});
    }
    auto sub = probe->create_subscription<Cloud>("output", rclcpp::SensorDataQoS(),
      [&received](Cloud::ConstSharedPtr msg) { received = msg; });
    auto points = probe->create_publisher<Cloud>("input", rclcpp::SensorDataQoS());
    auto motion = probe->create_publisher<Twist>("twist", rclcpp::QoS(100));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(component.get_node_base_interface()); executor.add_node(probe);
    if (chain) {
      executor.add_node(first->get_node_base_interface());
      executor.add_node(last->get_node_base_interface());
    }
    auto wait = [&](auto predicate) {
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!predicate() && std::chrono::steady_clock::now() < deadline) {
          executor.spin_some(10ms); std::this_thread::sleep_for(10ms);
        }
        return predicate();
      };
    ASSERT_TRUE(wait([&]() {return points->get_subscription_count() == 1 &&
      motion->get_subscription_count() == 1 && sub->get_publisher_count() == 1 &&
      (!chain || (probe->count_publishers("/atlas_deskew_test/extended") == 1 &&
        probe->count_subscribers("/atlas_deskew_test/extended") == 2 &&
        probe->count_publishers("/atlas_deskew_test/rectified") == 1 &&
        probe->count_subscribers("/atlas_deskew_test/rectified") == 2));}));

    tf2_ros::StaticTransformBroadcaster broadcaster(probe);
    geometry_msgs::msg::TransformStamped identity;
    identity.header.frame_id = base_frame; identity.child_frame_id = cloud_frame;
    identity.header.stamp = probe->now(); identity.transform.rotation.w = 1.0;
    broadcaster.sendTransform(identity);
    // Uses the same singleton provider as the dynamically loaded upstream component.
    managed_transform_buffer::ManagedTransformBuffer preflight;
    auto transform = preflight.getTransform<Eigen::Matrix4f>(base_frame, cloud_frame,
      probe->now(), rclcpp::Duration::from_seconds(2.0), probe->get_logger());
    ASSERT_TRUE(transform.has_value()) << "No cloud sent: managed TF preflight failed";
    ASSERT_TRUE(transform->isApprox(Eigen::Matrix4f::Identity(), 1e-6F));

    if (scenario != "Missing") {
      Twist twist; twist.header.frame_id = base_frame;
      if (scenario == "Stale") {
        twist.header.stamp.sec = 99; twist.header.stamp.nanosec = 500000000;
        twist.twist.twist.linear.x = 1.0; motion->publish(twist);
      } else {
        twist.header.stamp.sec = 100;
        twist.twist.twist.linear.x = scenario == "Forward" ? 1.0 : 0.0;
        motion->publish(twist);
        twist.header.stamp.nanosec = 50000000; motion->publish(twist);
      }
      // Reliable DDS acknowledgment gates cloud delivery. The upstream polling subscriber
      // takes all queued samples directly from its ROS subscription in the cloud callback.
      ASSERT_TRUE(motion->wait_for_all_acked(2s)) << "Motion delivery timed out";
      executor.spin_some(10ms);
    }
    const auto input = chain ? livox_cloud() : cloud();
    auto next_send = std::chrono::steady_clock::now();
    ASSERT_TRUE(wait([&]() {
      if ((!received || (chain && (!extended_received || !rectified_received))) &&
        std::chrono::steady_clock::now() >= next_send) {
        points->publish(input); next_send = std::chrono::steady_clock::now() + 100ms;
      }
      return received && (!chain || (extended_received && rectified_received));
    })) << "Corrector output timed out";
    const float second_x = scenario == "Forward" ? 1.05F : 1.0F;
    if (chain) {
      const auto expected_extended = expected_extended_cloud();
      verify(*extended_received, expected_extended, 1.0F);
      verify(*rectified_received, expected_extended, second_x);
      verify_compact(*received, input, second_x);
      // The compact component must copy the corrected 16-byte prefix of every point.
      ASSERT_EQ(received->data.size(), 32U);
      ASSERT_EQ(rectified_received->data.size(), 64U);
      for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 16; ++j) {
          EXPECT_EQ(received->data[i * 16 + j], rectified_received->data[i * 32 + j]);
        }
      }
    } else {
      verify(*received, input, second_x);
    }
  }
};
TEST_F(DistortionCorrector, Forward) { run("Forward"); }
TEST_F(DistortionCorrector, Zero) { run("Zero"); }
TEST_F(DistortionCorrector, Missing) { run("Missing"); }
TEST_F(DistortionCorrector, Stale) { run("Stale"); }
class EndToEnd : public DistortionCorrector {};
TEST_F(EndToEnd, Forward) { run("Forward", true); }
TEST_F(EndToEnd, Zero) { run("Zero", true); }
TEST_F(EndToEnd, Missing) { run("Missing", true); }
TEST_F(EndToEnd, Stale) { run("Stale", true); }
}  // namespace
