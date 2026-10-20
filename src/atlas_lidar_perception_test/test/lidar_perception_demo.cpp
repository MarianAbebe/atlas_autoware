#include <ament_index_cpp/get_package_prefix.hpp>
#include <class_loader/class_loader.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using Cloud = sensor_msgs::msg::PointCloud2;
using Objects = autoware_perception_msgs::msg::DetectedObjects;
using Twist = geometry_msgs::msg::TwistWithCovarianceStamped;
constexpr char kNamespace[] = "/atlas_lidar_perception_demo";
constexpr char kBaseFrame[] = "atlas_perception_base";
constexpr char kLidarFrame[] = "atlas_perception_lidar";

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template<typename T>
void put(Cloud & cloud, const size_t offset, const T value)
{
  require(offset + sizeof(T) <= cloud.data.size(), "Synthetic cloud write out of bounds");
  std::memcpy(cloud.data.data() + offset, &value, sizeof(T));
}

template<typename T>
T get(const Cloud & cloud, const size_t offset)
{
  require(offset + sizeof(T) <= cloud.data.size(), "Received cloud read out of bounds");
  T value{};
  std::memcpy(&value, cloud.data.data() + offset, sizeof(T));
  return value;
}

void add_field(Cloud & cloud, const std::string & name, const uint32_t offset,
  const uint8_t datatype)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  cloud.fields.push_back(field);
}

struct Obstacle
{
  float x;
  float y;
};
constexpr std::array<Obstacle, 2> kObstacles{{{6.0F, 1.5F}, {10.0F, -2.5F}}};
constexpr size_t kPointsPerObstacle = 9U * 7U * 5U;

Cloud make_livox_cloud(const builtin_interfaces::msg::Time & stamp)
{
  using Field = sensor_msgs::msg::PointField;
  Cloud cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = kLidarFrame;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(kPointsPerObstacle * kObstacles.size());
  cloud.point_step = 26;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_dense = true;
  add_field(cloud, "x", 0, Field::FLOAT32);
  add_field(cloud, "y", 4, Field::FLOAT32);
  add_field(cloud, "z", 8, Field::FLOAT32);
  add_field(cloud, "intensity", 12, Field::FLOAT32);
  add_field(cloud, "tag", 16, Field::UINT8);
  add_field(cloud, "line", 17, Field::UINT8);
  add_field(cloud, "timestamp", 18, Field::FLOAT64);
  cloud.data.resize(cloud.row_step);

  const int64_t stamp_ns = rclcpp::Time(stamp).nanoseconds();
  const double absolute_ns = static_cast<double>(stamp_ns);
  require(static_cast<int64_t>(absolute_ns) == stamp_ns,
    "Synthetic epoch timestamp is not exactly representable in Livox FLOAT64");
  size_t index = 0;
  for (const auto & obstacle : kObstacles) {
    for (size_t iz = 0; iz < 5; ++iz) {
      const float z = 0.2F + static_cast<float>(iz) * 0.3F;
      for (size_t iy = 0; iy < 7; ++iy) {
        const float y = obstacle.y - 0.3F + static_cast<float>(iy) * 0.1F;
        for (size_t ix = 0; ix < 9; ++ix, ++index) {
          const float x = obstacle.x - 0.4F + static_cast<float>(ix) * 0.1F;
          const size_t offset = index * cloud.point_step;
          put<float>(cloud, offset, x);
          put<float>(cloud, offset + 4, y);
          put<float>(cloud, offset + 8, z);
          put<float>(cloud, offset + 12, 25.0F + static_cast<float>(index % 80));
          put<uint8_t>(cloud, offset + 16, 0);
          put<uint8_t>(cloud, offset + 17, 0);
          put<double>(cloud, offset + 18, absolute_ns);
        }
      }
    }
  }
  return cloud;
}

void verify_fields(const Cloud & cloud, const bool compact)
{
  using Field = sensor_msgs::msg::PointField;
  const std::vector<std::string> expected_names = compact
    ? std::vector<std::string>{"x", "y", "z", "intensity", "return_type", "channel"}
    : std::vector<std::string>{"x", "y", "z", "intensity", "return_type", "channel",
      "azimuth", "elevation", "distance", "time_stamp"};
  const std::vector<uint32_t> expected_offsets = compact
    ? std::vector<uint32_t>{0, 4, 8, 12, 13, 14}
    : std::vector<uint32_t>{0, 4, 8, 12, 13, 14, 16, 20, 24, 28};
  const std::vector<uint8_t> expected_types = compact
    ? std::vector<uint8_t>{Field::FLOAT32, Field::FLOAT32, Field::FLOAT32,
      Field::UINT8, Field::UINT8, Field::UINT16}
    : std::vector<uint8_t>{Field::FLOAT32, Field::FLOAT32, Field::FLOAT32, Field::UINT8,
      Field::UINT8, Field::UINT16, Field::FLOAT32, Field::FLOAT32, Field::FLOAT32,
      Field::UINT32};
  const size_t step = compact ? 16U : 32U;
  require(cloud.fields.size() == expected_names.size() && cloud.point_step == step &&
    cloud.height == 1 && cloud.row_step == cloud.width * step &&
    cloud.data.size() == cloud.row_step && !cloud.is_bigendian,
    compact ? "XYZIRC cloud bounds or stride mismatch" : "XYZIRCAEDT cloud bounds or stride mismatch");
  for (size_t i = 0; i < expected_names.size(); ++i) {
    const auto & field = cloud.fields[i];
    require(field.name == expected_names[i] && field.offset == expected_offsets[i] &&
      field.datatype == expected_types[i] && field.count == 1,
      "PointCloud2 layout mismatch at field " + expected_names[i]);
  }
}

struct Observations
{
  std::mutex mutex;
  std::condition_variable condition;
  Cloud::ConstSharedPtr extended;
  Cloud::ConstSharedPtr rectified;
  Cloud::ConstSharedPtr compact;
  Cloud::ConstSharedPtr clusters;
  Objects::ConstSharedPtr objects;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr markers;

  template<typename Predicate>
  bool wait(Predicate predicate, const std::chrono::seconds timeout = 12s)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, timeout, predicate);
  }
};

int demonstration()
{
  auto driver = std::make_shared<rclcpp::Node>("lidar_perception_demo", kNamespace,
    rclcpp::NodeOptions().use_intra_process_comms(false));
  const int hold_seconds = driver->declare_parameter<int>("hold_seconds", 0);
  require(hold_seconds >= 0 && hold_seconds <= 60, "hold_seconds must be in [0, 60]");
  const auto sensor_prefix = ament_index_cpp::get_package_prefix("atlas_sensor_kit");
  const auto deskew_prefix = ament_index_cpp::get_package_prefix("atlas_distortion_corrector_test");
  const auto detector_prefix = ament_index_cpp::get_package_prefix(
    "autoware_euclidean_cluster_object_detector");
  const std::string converter_library = sensor_prefix + "/lib/libpointcloud_components.so";
  const std::string deskew_library = deskew_prefix + "/lib/libpinned_distortion_corrector.so";
  const std::string detector_library = detector_prefix +
    "/lib/libautoware_euclidean_cluster_object_detector_node_core.so";

  class_loader::ClassLoader converter_loader(converter_library);
  class_loader::ClassLoader deskew_loader(deskew_library);
  class_loader::ClassLoader detector_loader(detector_library);
  const std::string livox_name =
    "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::LivoxToExtendedComponent>";
  const std::string compact_name =
    "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::ExtendedToCompactComponent>";
  const std::string deskew_name = "rclcpp_components::NodeFactoryTemplate<"
    "autoware::pointcloud_preprocessor::DistortionCorrectorComponent>";
  const std::string detector_name = "rclcpp_components::NodeFactoryTemplate<"
    "autoware::euclidean_cluster::EuclideanClusterNode>";
  const auto converter_factories = converter_loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  const auto deskew_factories = deskew_loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  const auto detector_factories = detector_loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  require(std::count(converter_factories.begin(), converter_factories.end(), livox_name) == 1 &&
    std::count(converter_factories.begin(), converter_factories.end(), compact_name) == 1,
    "Expected one registration for each Atlas point-cloud converter");
  require(std::count(deskew_factories.begin(), deskew_factories.end(), deskew_name) == 1,
    "Expected exactly one pinned distortion-corrector factory");
  require(std::count(detector_factories.begin(), detector_factories.end(), detector_name) == 1,
    "Expected exactly one pinned Autoware EuclideanClusterNode factory");

  auto livox_factory = converter_loader.createInstance<rclcpp_components::NodeFactory>(livox_name);
  auto compact_factory = converter_loader.createInstance<rclcpp_components::NodeFactory>(compact_name);
  auto deskew_factory = deskew_loader.createInstance<rclcpp_components::NodeFactory>(deskew_name);
  auto detector_factory = detector_loader.createInstance<rclcpp_components::NodeFactory>(detector_name);
  const auto options = [](std::vector<std::string> remaps) {
    rclcpp::NodeOptions result;
    result.use_intra_process_comms(false).use_global_arguments(false);
    std::vector<std::string> args{"--ros-args", "-r", std::string("__ns:=") + kNamespace};
    args.insert(args.end(), remaps.begin(), remaps.end());
    result.arguments(args);
    return result;
  };
  auto livox_component = livox_factory->create_node_instance(options({
    "-r", "~/input/pointcloud:=/atlas_lidar_perception_demo/livox",
    "-r", "~/output/pointcloud:=/atlas_lidar_perception_demo/extended"}));
  auto deskew_options = options({
    "-r", "~/input/pointcloud:=/atlas_lidar_perception_demo/extended",
    "-r", "~/input/twist:=/atlas_lidar_perception_demo/twist",
    "-r", "~/output/pointcloud:=/atlas_lidar_perception_demo/rectified"});
  deskew_options.parameter_overrides({
    rclcpp::Parameter("base_frame", kBaseFrame),
    rclcpp::Parameter("use_imu", false),
    rclcpp::Parameter("use_3d_distortion_correction", false),
    rclcpp::Parameter("update_azimuth_and_distance", false),
    rclcpp::Parameter("processing_time_threshold_sec", 0.01),
    rclcpp::Parameter("timestamp_mismatch_fraction_threshold", 0.01)});
  auto deskew_component = deskew_factory->create_node_instance(deskew_options);
  auto compact_component = compact_factory->create_node_instance(options({
    "-r", "~/input/pointcloud:=/atlas_lidar_perception_demo/rectified",
    "-r", "~/output/pointcloud:=/atlas_lidar_perception_demo/compact"}));
  auto detector_options = options({
    "-r", "input:=/atlas_lidar_perception_demo/compact",
    "-r", "output:=/atlas_lidar_perception_demo/objects"});
  detector_options.parameter_overrides({
    rclcpp::Parameter("use_height", false),
    rclcpp::Parameter("min_cluster_size", 10),
    rclcpp::Parameter("max_cluster_size", 1000),
    rclcpp::Parameter("tolerance", 0.22)});
  auto detector_component = detector_factory->create_node_instance(detector_options);

  auto input_pub = driver->create_publisher<Cloud>("livox", rclcpp::SensorDataQoS());
  auto twist_pub = driver->create_publisher<Twist>("twist", rclcpp::QoS(100));
  auto marker_pub = driver->create_publisher<visualization_msgs::msg::MarkerArray>(
    "detected_centers", rclcpp::QoS(1));
  Observations seen;
  auto extended_sub = driver->create_subscription<Cloud>("extended", rclcpp::SensorDataQoS(),
    [&](Cloud::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(seen.mutex);
      seen.extended = std::move(msg);
      seen.condition.notify_all();
    });
  auto rectified_sub = driver->create_subscription<Cloud>("rectified", rclcpp::SensorDataQoS(),
    [&](Cloud::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(seen.mutex);
      seen.rectified = std::move(msg);
      seen.condition.notify_all();
    });
  auto compact_sub = driver->create_subscription<Cloud>("compact", rclcpp::SensorDataQoS(),
    [&](Cloud::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(seen.mutex);
      seen.compact = std::move(msg);
      seen.condition.notify_all();
    });
  auto clusters_sub = driver->create_subscription<Cloud>("debug/clusters", rclcpp::QoS(1),
    [&](Cloud::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(seen.mutex);
      seen.clusters = std::move(msg);
      seen.condition.notify_all();
    });
  auto markers_sub = driver->create_subscription<visualization_msgs::msg::MarkerArray>(
    "detected_centers", rclcpp::QoS(1), [&](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(seen.mutex);
      seen.markers = std::move(msg);
      seen.condition.notify_all();
    });
  auto objects_sub = driver->create_subscription<Objects>("objects", rclcpp::QoS(1),
    [&](Objects::ConstSharedPtr msg) {
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker clear;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);
      for (size_t i = 0; i < msg->objects.size(); ++i) {
        const auto & object = msg->objects[i];
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "centers";
        marker.id = static_cast<int32_t>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = object.kinematics.pose_with_covariance.pose;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25;
        marker.color.a = 0.9F;
        marker.color.r = i == 0 ? 1.0F : 0.1F;
        marker.color.g = 0.2F;
        marker.color.b = i == 0 ? 0.1F : 1.0F;
        marker.lifetime.sec = 2;
        marker.lifetime.nanosec = 0;
        markers.markers.push_back(marker);
      }
      marker_pub->publish(markers);
      std::lock_guard<std::mutex> lock(seen.mutex);
      seen.objects = std::move(msg);
      seen.condition.notify_all();
    });

  tf2_ros::StaticTransformBroadcaster static_tf(driver);
  tf2_ros::Buffer tf_buffer(driver->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(livox_component.get_node_base_interface());
  executor.add_node(deskew_component.get_node_base_interface());
  executor.add_node(compact_component.get_node_base_interface());
  executor.add_node(detector_component.get_node_base_interface());
  executor.add_node(driver);
  std::thread spin_thread([&executor] {executor.spin();});
  const auto stop = [&] {
    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
  };
  try {
    const auto discovery_deadline = std::chrono::steady_clock::now() + 15s;
    while (input_pub->get_subscription_count() == 0 || twist_pub->get_subscription_count() == 0 ||
      driver->count_publishers("extended") == 0 || driver->count_publishers("rectified") == 0 ||
      driver->count_publishers("compact") == 0 || driver->count_publishers("objects") == 0 ||
      driver->count_publishers("debug/clusters") == 0 ||
      driver->count_subscribers("detected_centers") == 0) {
      require(std::chrono::steady_clock::now() < discovery_deadline,
        "ROS component/topic discovery timed out");
      std::this_thread::sleep_for(20ms);
    }
    geometry_msgs::msg::TransformStamped identity;
    identity.header.stamp = driver->now();
    identity.header.frame_id = kBaseFrame;
    identity.child_frame_id = kLidarFrame;
    identity.transform.rotation.w = 1.0;
    static_tf.sendTransform(identity);
    const auto test_tf = tf_buffer.lookupTransform(kBaseFrame, kLidarFrame,
      tf2::TimePointZero, tf2::durationFromSec(5.0));
    require(std::abs(test_tf.transform.translation.x) < 1e-9 &&
      std::abs(test_tf.transform.translation.y) < 1e-9 &&
      std::abs(test_tf.transform.translation.z) < 1e-9 &&
      std::abs(test_tf.transform.rotation.w - 1.0) < 1e-9,
      "Test-only static TF is not identity");

    const int64_t start_ns = rclcpp::Time(driver->now()).nanoseconds();
    std::vector<std::array<double, 3>> first_centers;
    for (size_t pass = 0; pass < 3; ++pass) {
      builtin_interfaces::msg::Time stamp;
      const int64_t scan_ns = ((start_ns + static_cast<int64_t>(pass) * 100000000LL) / 256LL) * 256LL;
      stamp.sec = static_cast<int32_t>(scan_ns / 1000000000LL);
      stamp.nanosec = static_cast<uint32_t>(scan_ns % 1000000000LL);
      const Cloud raw = make_livox_cloud(stamp);
      {
        std::lock_guard<std::mutex> lock(seen.mutex);
        seen.extended.reset(); seen.rectified.reset(); seen.compact.reset();
        seen.clusters.reset(); seen.objects.reset(); seen.markers.reset();
      }

      Twist first, last;
      first.header.stamp = stamp;
      first.header.frame_id = kBaseFrame;
      last = first;
      last.header.stamp = rclcpp::Time(stamp) + rclcpp::Duration::from_seconds(0.01);
      twist_pub->publish(first);
      twist_pub->publish(last);
      require(twist_pub->wait_for_all_acked(2s), "Synthetic twist delivery timed out");
      std::this_thread::sleep_for(50ms);
      input_pub->publish(raw);

      require(seen.wait([&] {
        const auto same_stamp = [&stamp](const auto & msg) {
          return msg && msg->header.stamp == stamp;
        };
        return same_stamp(seen.extended) && same_stamp(seen.rectified) &&
          same_stamp(seen.compact) && same_stamp(seen.clusters) && same_stamp(seen.objects) &&
          seen.markers && seen.markers->markers.size() > 1 &&
          seen.markers->markers[1].header.stamp == stamp;
      }), "Preprocessing or detector output timed out");
      Cloud::ConstSharedPtr extended, rectified, compact, clusters;
      Objects::ConstSharedPtr objects;
      visualization_msgs::msg::MarkerArray::ConstSharedPtr markers;
      {
        std::lock_guard<std::mutex> lock(seen.mutex);
        extended = seen.extended; rectified = seen.rectified; compact = seen.compact;
        clusters = seen.clusters; objects = seen.objects; markers = seen.markers;
      }

      require(extended->header == raw.header && rectified->header == raw.header &&
        compact->header == raw.header && clusters->header == raw.header &&
        objects->header == raw.header, "Cloud or detection header was not preserved");
      require(extended->width == raw.width && rectified->width == raw.width &&
        compact->width == raw.width, "Preprocessing changed point count");
      verify_fields(*extended, false);
      verify_fields(*rectified, false);
      verify_fields(*compact, true);
      for (size_t i = 0; i < raw.width; ++i) {
        const size_t raw_offset = i * raw.point_step;
        const size_t extended_offset = i * extended->point_step;
        const size_t rectified_offset = i * rectified->point_step;
        const size_t compact_offset = i * compact->point_step;
        require(get<uint32_t>(*extended, extended_offset + 28) == 0 &&
          get<uint32_t>(*rectified, rectified_offset + 28) == 0,
          "Zero point-time offset changed during conversion or deskew");
        for (size_t axis = 0; axis < 3; ++axis) {
          const size_t axis_offset = axis * sizeof(float);
          require(get<float>(*extended, extended_offset + axis_offset) ==
              get<float>(raw, raw_offset + axis_offset) &&
            get<float>(*rectified, rectified_offset + axis_offset) ==
              get<float>(*extended, extended_offset + axis_offset) &&
            get<float>(*compact, compact_offset + axis_offset) ==
              get<float>(*rectified, rectified_offset + axis_offset),
            "A preprocessing component changed synthetic XYZ unexpectedly");
        }
        require(get<uint8_t>(*compact, compact_offset + 12) ==
            static_cast<uint8_t>(get<float>(raw, raw_offset + 12)) &&
          get<uint8_t>(*compact, compact_offset + 13) == 0 &&
          get<uint16_t>(*compact, compact_offset + 14) == 0,
          "Intensity or provisional return/channel metadata mismatch");
      }
      require(clusters->width == raw.width && clusters->fields.size() == 4 &&
        clusters->fields[0].name == "x" && clusters->fields[1].name == "y" &&
        clusters->fields[2].name == "z" && clusters->fields[3].name == "rgb",
        "Pinned detector debug cluster cloud is malformed");
      require(objects->objects.size() == kObstacles.size(), "Expected exactly two detected objects");
      require(markers->markers.size() == kObstacles.size() + 1,
        "RViz center-marker output does not match detected objects");

      std::vector<std::array<double, 3>> centers;
      for (const auto & object : objects->objects) {
        require(object.classification.size() == 1 &&
          object.classification.front().label ==
            autoware_perception_msgs::msg::ObjectClassification::UNKNOWN,
          "Unexpected synthetic point-cloud classification");
        const auto & p = object.kinematics.pose_with_covariance.pose.position;
        centers.push_back({p.x, p.y, p.z});
      }
      std::sort(centers.begin(), centers.end());
      for (size_t i = 0; i < centers.size(); ++i) {
        const std::array<double, 3> expected{
          static_cast<double>(kObstacles[i].x), static_cast<double>(kObstacles[i].y), 0.8};
        const double error = std::sqrt(
          std::pow(centers[i][0] - expected[0], 2) +
          std::pow(centers[i][1] - expected[1], 2) +
          std::pow(centers[i][2] - expected[2], 2));
        std::cout << "scan " << pass + 1 << " obstacle " << i + 1 << " detected at ("
          << centers[i][0] << ", " << centers[i][1] << ", " << centers[i][2]
          << ") m; error=" << error << " m" << std::endl;
        require(std::isfinite(error) && error < 0.03, "Detected obstacle center error exceeds 0.03 m");
      }
      if (pass == 0) {
        first_centers = centers;
      } else {
        for (size_t i = 0; i < centers.size(); ++i) {
          for (size_t axis = 0; axis < 3; ++axis) {
            require(std::abs(centers[i][axis] - first_centers[i][axis]) < 1e-6,
              "Repeated synthetic scan changed the detected center");
          }
        }
      }
      std::cout << "PASS scan " << pass + 1 << ": " << compact->width
        << " XYZIRC points through the registered pinned Euclidean detector; 2 objects.\n";
    }
    const auto display_until = std::chrono::steady_clock::now() +
      std::chrono::seconds(hold_seconds);
    size_t display_scan = 0;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < display_until) {
      const int64_t now_ns = rclcpp::Time(driver->now()).nanoseconds();
      const int64_t scan_ns = (now_ns / 256LL) * 256LL;
      builtin_interfaces::msg::Time stamp;
      stamp.sec = static_cast<int32_t>(scan_ns / 1000000000LL);
      stamp.nanosec = static_cast<uint32_t>(scan_ns % 1000000000LL);
      {
        std::lock_guard<std::mutex> lock(seen.mutex);
        seen.extended.reset(); seen.rectified.reset(); seen.compact.reset();
        seen.clusters.reset(); seen.objects.reset(); seen.markers.reset();
      }
      Twist first, last;
      first.header.stamp = stamp;
      first.header.frame_id = kBaseFrame;
      last = first;
      last.header.stamp = rclcpp::Time(stamp) + rclcpp::Duration::from_seconds(0.01);
      twist_pub->publish(first);
      twist_pub->publish(last);
      require(twist_pub->wait_for_all_acked(2s), "RViz-mode twist delivery timed out");
      std::this_thread::sleep_for(50ms);
      input_pub->publish(make_livox_cloud(stamp));
      require(seen.wait([&] {
        const auto same_stamp = [&stamp](const auto & msg) {
          return msg && msg->header.stamp == stamp;
        };
        return same_stamp(seen.extended) && same_stamp(seen.rectified) &&
          same_stamp(seen.compact) && same_stamp(seen.clusters) && same_stamp(seen.objects) &&
          seen.markers && seen.markers->markers.size() > 1 &&
          seen.markers->markers[1].header.stamp == stamp;
      }), "RViz-mode detector output timed out");
      ++display_scan;
      std::this_thread::sleep_for(250ms);
    }
    if (display_scan > 0) {
      std::cout << "RViz hold published " << display_scan << " additional synthetic scans.\n";
    }
    stop();
  } catch (...) {
    stop();
    throw;
  }
  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    status = demonstration();
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << std::endl;
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
