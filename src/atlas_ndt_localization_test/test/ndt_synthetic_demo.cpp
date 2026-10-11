#include "stub_pcd_loader.hpp"
#include "test_util.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <class_loader/class_loader.hpp>
#include <managed_transform_buffer/managed_transform_buffer.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <cstring>
#include <optional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Cloud = sensor_msgs::msg::PointCloud2;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Twist = geometry_msgs::msg::TwistWithCovarianceStamped;
constexpr char base_frame[] = "atlas_ndt_test_base";
constexpr char scan_frame[] = "atlas_ndt_test_lidar";
constexpr char estimate_frame[] = "atlas_ndt_test_estimate";

void require(bool ok, const std::string & message)
{
  if (!ok) throw std::runtime_error(message);
}

template<class T> void put(Cloud & cloud, size_t at, T value)
{
  require(at + sizeof(T) <= cloud.data.size(), "Synthetic field write out of bounds");
  std::memcpy(cloud.data.data() + at, &value, sizeof(T));
}
template<class T> T get(const Cloud & cloud, size_t at)
{
  require(at + sizeof(T) <= cloud.data.size(), "Received field read out of bounds");
  T value; std::memcpy(&value, cloud.data.data() + at, sizeof(T)); return value;
}

// Epoch nanoseconds stored in FLOAT64 have a 256 ns ULP at contemporary dates.
// Use exactly representable header and point times; never disguise quantization.
uint32_t acquisition_offset(size_t i, size_t count)
{
  return static_cast<uint32_t>((i * 195312ULL / (count - 1)) * 256ULL);
}
Cloud livox_acquisition(const Cloud & reference)
{
  pcl::PointCloud<pcl::PointXYZ> points; pcl::fromROSMsg(reference, points);
  require(points.size() > 1, "Synthetic geometry needs multiple points");
  Cloud cloud; cloud.header = reference.header;
  cloud.height = 1; cloud.width = points.size(); cloud.point_step = 26;
  cloud.row_step = cloud.width * cloud.point_step; cloud.is_dense = true;
  using F = sensor_msgs::msg::PointField;
  const std::vector<std::string> names{"x","y","z","intensity","tag","line","timestamp"};
  const std::vector<uint32_t> offsets{0,4,8,12,16,17,18};
  const std::vector<uint8_t> types{F::FLOAT32,F::FLOAT32,F::FLOAT32,F::FLOAT32,F::UINT8,F::UINT8,F::FLOAT64};
  for (size_t i = 0; i < names.size(); ++i) {
    F field; field.name = names[i]; field.offset = offsets[i]; field.datatype = types[i];
    field.count = 1; cloud.fields.push_back(field);
  }
  cloud.data.resize(cloud.row_step);
  const auto start = rclcpp::Time(cloud.header.stamp).nanoseconds();
  for (size_t i = 0; i < points.size(); ++i) {
    const auto dt = acquisition_offset(i, points.size());
    const auto absolute = start + dt;
    const double timestamp = static_cast<double>(absolute);
    require(static_cast<int64_t>(timestamp) == absolute,
      "Synthetic absolute timestamp is not exactly representable in FLOAT64");
    // A fixed world point moves backwards in the sensor frame during +1 m/s acquisition.
    put<float>(cloud, i*26, points[i].x - static_cast<float>(dt * 1e-9));
    put<float>(cloud, i*26+4, points[i].y); put<float>(cloud, i*26+8, points[i].z);
    put<float>(cloud, i*26+12, i%2 == 0 ? 21.5F : 87.25F);
    put<uint8_t>(cloud, i*26+16, 17); put<uint8_t>(cloud, i*26+17, 5);
    put<double>(cloud, i*26+18, timestamp);
  }
  return cloud;
}

void verify_preprocessing(const Cloud & extended, const Cloud & rectified,
  const Cloud & compact, const Cloud & raw, const Cloud & reference)
{
  using F = sensor_msgs::msg::PointField;
  const std::vector<std::string> raw_names{"x","y","z","intensity","tag","line","timestamp"};
  const std::vector<uint32_t> raw_offsets{0,4,8,12,16,17,18};
  const std::vector<uint8_t> raw_types{F::FLOAT32,F::FLOAT32,F::FLOAT32,F::FLOAT32,
    F::UINT8,F::UINT8,F::FLOAT64};
  require(!raw.is_bigendian && raw.height == 1 && raw.point_step == 26 &&
    raw.row_step == raw.width * 26U && raw.data.size() == raw.row_step &&
    raw.fields.size() == raw_names.size(), "Livox PointCloud2 input schema mismatch");
  for (size_t j = 0; j < raw_names.size(); ++j) {
    require(raw.fields[j].name == raw_names[j] && raw.fields[j].offset == raw_offsets[j] &&
      raw.fields[j].datatype == raw_types[j] && raw.fields[j].count == 1,
      "Livox field contract mismatch at " + raw_names[j]);
  }
  const std::vector<std::string> names{"x","y","z","intensity","return_type","channel",
    "azimuth","elevation","distance","time_stamp"};
  const std::vector<uint32_t> offsets{0,4,8,12,13,14,16,20,24,28};
  const std::vector<uint8_t> types{F::FLOAT32,F::FLOAT32,F::FLOAT32,F::UINT8,F::UINT8,
    F::UINT16,F::FLOAT32,F::FLOAT32,F::FLOAT32,F::UINT32};
  for (const auto * cloud : {&extended, &rectified, &compact}) {
    const bool small = cloud == &compact; const uint32_t step = small ? 16 : 32;
    require(cloud->header == raw.header, "Preprocessing changed acquisition header");
    require(cloud->width == raw.width && cloud->height == 1 && cloud->point_step == step &&
      cloud->row_step == raw.width * step && cloud->data.size() == cloud->row_step &&
      !cloud->is_bigendian && cloud->is_dense, "Preprocessing count/stride/bounds mismatch");
    require(cloud->fields.size() == (small ? 6U : 10U), "Wrong field count");
    for (size_t j = 0; j < cloud->fields.size(); ++j) {
      const auto & f = cloud->fields[j];
      require(f.name == names[j] && f.offset == offsets[j] && f.datatype == types[j] &&
        f.count == 1, "Point layout mismatch at " + names[j]);
    }
  }
  pcl::PointCloud<pcl::PointXYZ> expected; pcl::fromROSMsg(reference, expected);
  const auto start = rclcpp::Time(raw.header.stamp).nanoseconds();
  for (size_t i = 0; i < raw.width; ++i) {
    const int64_t absolute_ns = start + acquisition_offset(i, raw.width);
    const double absolute_timestamp = get<double>(raw, i * 26 + 18);
    require(absolute_timestamp == static_cast<double>(absolute_ns) &&
      static_cast<int64_t>(absolute_timestamp) == absolute_ns,
      "Livox absolute FLOAT64 timestamp mismatch or precision loss");
    require(get<uint32_t>(extended,i*32+28) == acquisition_offset(i,raw.width) &&
      get<uint32_t>(rectified,i*32+28) == acquisition_offset(i,raw.width), "Point timing mismatch");
    for (size_t j = 0; j < 12; ++j) {
      require(extended.data[i*32+j] == raw.data[i*26+j], "First converter changed XYZ");
      require(compact.data[i*16+j] == rectified.data[i*32+j], "Compaction changed XYZ");
    }
    for (size_t j = 12; j < 32; ++j) {
      require(extended.data[i*32+j] == rectified.data[i*32+j], "Deskew changed non-XYZ bytes");
      if (j < 16) require(compact.data[i*16+j] == rectified.data[i*32+j], "Compaction changed metadata");
    }
    require(get<uint8_t>(compact,i*16+12) == (i%2 == 0 ? 22 : 87) &&
      get<uint8_t>(compact,i*16+13) == 0 && get<uint16_t>(compact,i*16+14) == 0,
      "Intensity or provisional metadata mismatch");
    require(std::abs(get<float>(compact,i*16) - expected[i].x) < 1e-5F &&
      std::abs(get<float>(compact,i*16+4) - expected[i].y) < 1e-5F &&
      std::abs(get<float>(compact,i*16+8) - expected[i].z) < 1e-5F,
      "Deskew did not recover matched scan geometry");
  }
  require(std::abs(get<float>(compact,(raw.width-1)*16) -
    get<float>(raw,(raw.width-1)*26)) > 0.049F, "No observable deskew displacement");
  std::cout << "Verified processed NDT input: " << compact.width <<
    " XYZIRC points, 16-byte stride, preserved timing/metadata; deskew displacement ~0.05 m" << std::endl;
}

struct Observations
{
  std::mutex mutex;
  std::condition_variable cv;
  Cloud::ConstSharedPtr map;
  Cloud::ConstSharedPtr aligned;
  Pose::ConstSharedPtr pose;
  Cloud::ConstSharedPtr extended, rectified, compact;
  bool wait(const std::function<bool()> & ready, std::chrono::seconds timeout = 15s)
  {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, timeout, ready);
  }
};

int demonstration()
{
  const auto share = ament_index_cpp::get_package_share_directory("autoware_ndt_scan_matcher");
  auto * yaml = rcl_yaml_node_struct_init(rcl_get_default_allocator());
  require(yaml != nullptr, "Cannot allocate YAML parser");
  const bool parsed = rcl_parse_yaml_file((share + "/config/ndt_scan_matcher.param.yaml").c_str(), yaml);
  if (!parsed) {
    rcl_yaml_node_struct_fini(yaml);
    throw std::runtime_error("Cannot parse upstream NDT parameters");
  }
  std::vector<rclcpp::Parameter> parameters;
  for (const auto & entry : rclcpp::parameter_map_from(yaml, "")) {
    parameters.insert(parameters.end(), entry.second.begin(), entry.second.end());
  }
  rcl_yaml_node_struct_fini(yaml);
  auto probe = std::make_shared<rclcpp::Node>("synthetic_driver", "/atlas_ndt_demo",
    rclcpp::NodeOptions().use_intra_process_comms(false));
  const bool pipeline = probe->declare_parameter<bool>("pipeline", false);
  const auto override_parameter = [&](const rclcpp::Parameter & value) {
    parameters.erase(std::remove_if(parameters.begin(), parameters.end(), [&](const auto & p) {
      return p.get_name() == value.get_name();
    }), parameters.end());
    parameters.push_back(value);
  };
  override_parameter(rclcpp::Parameter("frame.base_frame", base_frame));
  override_parameter(rclcpp::Parameter("frame.ndt_base_frame", estimate_frame));
  override_parameter(rclcpp::Parameter("dynamic_map_loading.publish_loaded_map", true));
  // All solver settings and convergence thresholds remain upstream defaults.
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false).use_global_arguments(false);
  options.parameter_overrides(parameters);
  std::vector<std::string> ndt_arguments{"--ros-args", "-r", "__ns:=/atlas_ndt_demo", "-r",
    "pcd_loader_service:=/pcd_loader_service"};
  if (pipeline) {
    ndt_arguments.insert(ndt_arguments.end(), {"-r", "points_raw:=/atlas_ndt_demo/compact"});
  }
  options.arguments(ndt_arguments);
  const auto library = ament_index_cpp::get_package_prefix("autoware_ndt_scan_matcher") +
    "/lib/libautoware_ndt_scan_matcher.so";
  class_loader::ClassLoader loader(library);
  const std::string factory_name =
    "rclcpp_components::NodeFactoryTemplate<autoware::ndt_scan_matcher::NDTScanMatcher>";
  const auto factories = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  require(std::count(factories.begin(), factories.end(), factory_name) == 1,
    "Expected exactly one registered pinned NDT factory");
  auto factory = loader.createInstance<rclcpp_components::NodeFactory>(factory_name);
  auto component = factory->create_node_instance(options);
  std::cout << "Loaded actual registered NDT component: " << library << std::endl;

  std::unique_ptr<class_loader::ClassLoader> converter_loader;
  std::unique_ptr<class_loader::ClassLoader> deskew_loader;
  std::shared_ptr<rclcpp_components::NodeFactory> livox_factory, compact_factory, deskew_factory;
  std::optional<rclcpp_components::NodeInstanceWrapper> livox_component, compact_component,
    deskew_component;
  if (pipeline) {
    const auto converter_library =
      ament_index_cpp::get_package_prefix("atlas_sensor_kit") +
      "/lib/libpointcloud_components.so";
    converter_loader = std::make_unique<class_loader::ClassLoader>(converter_library);
    const auto converter_classes = converter_loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    const std::string livox_name =
      "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::LivoxToExtendedComponent>";
    const std::string compact_name =
      "rclcpp_components::NodeFactoryTemplate<atlas_sensor_kit::ExtendedToCompactComponent>";
    require(std::count(converter_classes.begin(), converter_classes.end(), livox_name) == 1 &&
      std::count(converter_classes.begin(), converter_classes.end(), compact_name) == 1,
      "Expected exactly one registration for each Atlas converter");
    livox_factory = converter_loader->createInstance<rclcpp_components::NodeFactory>(livox_name);
    compact_factory = converter_loader->createInstance<rclcpp_components::NodeFactory>(compact_name);

    const auto deskew_library = ament_index_cpp::get_package_prefix(
      "atlas_distortion_corrector_test") + "/lib/libpinned_distortion_corrector.so";
    deskew_loader = std::make_unique<class_loader::ClassLoader>(deskew_library);
    const auto deskew_classes = deskew_loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    const std::string deskew_name =
      "rclcpp_components::NodeFactoryTemplate<autoware::pointcloud_preprocessor::DistortionCorrectorComponent>";
    require(std::count(deskew_classes.begin(), deskew_classes.end(), deskew_name) == 1,
      "Expected exactly one registered pinned distortion corrector");
    deskew_factory = deskew_loader->createInstance<rclcpp_components::NodeFactory>(deskew_name);

    rclcpp::NodeOptions livox_options, deskew_options, compact_options;
    for (auto * component_options : {&livox_options, &deskew_options, &compact_options}) {
      component_options->use_intra_process_comms(false);
      component_options->arguments({"--ros-args", "-r", "__ns:=/atlas_ndt_demo"});
    }
    livox_options.arguments({"--ros-args", "-r", "__ns:=/atlas_ndt_demo", "-r",
      "~/input/pointcloud:=/atlas_ndt_demo/livox", "-r",
      "~/output/pointcloud:=/atlas_ndt_demo/extended"});
    deskew_options.arguments({"--ros-args", "-r", "__ns:=/atlas_ndt_demo", "-r",
      "~/input/pointcloud:=/atlas_ndt_demo/extended", "-r",
      "~/input/twist:=/atlas_ndt_demo/twist", "-r",
      "~/output/pointcloud:=/atlas_ndt_demo/rectified"});
    compact_options.arguments({"--ros-args", "-r", "__ns:=/atlas_ndt_demo", "-r",
      "~/input/pointcloud:=/atlas_ndt_demo/rectified", "-r",
      "~/output/pointcloud:=/atlas_ndt_demo/compact"});
    deskew_options.parameter_overrides({rclcpp::Parameter("base_frame", base_frame),
      rclcpp::Parameter("use_imu", false),
      rclcpp::Parameter("use_3d_distortion_correction", false),
      rclcpp::Parameter("update_azimuth_and_distance", false),
      rclcpp::Parameter("processing_time_threshold_sec", 0.01),
      rclcpp::Parameter("timestamp_mismatch_fraction_threshold", 0.01)});
    livox_component.emplace(livox_factory->create_node_instance(livox_options));
    deskew_component.emplace(deskew_factory->create_node_instance(deskew_options));
    compact_component.emplace(compact_factory->create_node_instance(compact_options));
    std::cout << "Loaded actual registered Atlas converters and pinned deskewer" << std::endl;
  }

  auto map_loader = std::make_shared<StubPcdLoader>(); // Unchanged upstream mock service.
  const int hold_seconds = probe->declare_parameter<int>("hold_seconds", 0);
  require(hold_seconds >= 0 && hold_seconds <= 600, "hold_seconds must be in [0,600]");
  auto scans = probe->create_publisher<Cloud>(pipeline ? "livox" : "points_raw",
    rclcpp::SensorDataQoS());
  auto seeds = probe->create_publisher<Pose>("ekf_pose_with_covariance", 10);
  auto motion = probe->create_publisher<Twist>("twist", rclcpp::QoS(100));
  auto activate = probe->create_client<std_srvs::srv::SetBool>("trigger_node_srv");
  Observations observed;
  auto map_sub = probe->create_subscription<Cloud>("debug/loaded_pointcloud_map",
    rclcpp::QoS(1).transient_local(), [&](Cloud::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(observed.mutex);
      observed.map = msg; observed.cv.notify_all();
    });
  auto pose_sub = probe->create_subscription<Pose>("ndt_pose_with_covariance", 10,
    [&](Pose::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(observed.mutex);
      observed.pose = msg; observed.cv.notify_all();
    });
  auto aligned_sub = probe->create_subscription<Cloud>("points_aligned", 10,
    [&](Cloud::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(observed.mutex);
      observed.aligned = msg; observed.cv.notify_all();
    });
  rclcpp::Subscription<Cloud>::SharedPtr extended_sub, rectified_sub, compact_sub;
  if (pipeline) {
    extended_sub = probe->create_subscription<Cloud>("extended", rclcpp::SensorDataQoS(),
      [&](Cloud::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observed.mutex);
        observed.extended = msg; observed.cv.notify_all();
      });
    rectified_sub = probe->create_subscription<Cloud>("rectified", rclcpp::SensorDataQoS(),
      [&](Cloud::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observed.mutex);
        observed.rectified = msg; observed.cv.notify_all();
      });
    compact_sub = probe->create_subscription<Cloud>("compact", rclcpp::SensorDataQoS(),
      [&](Cloud::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observed.mutex);
        observed.compact = msg; observed.cv.notify_all();
      });
  }
  auto diagnostics_sub = probe->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10, [](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
      for (const auto & status : msg->status) {
        if (status.name.find("ndt_scan_matcher") == std::string::npos) continue;
        if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
          std::cerr << "NDT diagnostic: " << status.name << ": " << status.message << std::endl;
          for (const auto & kv : status.values) {
            std::cerr << "  " << kv.key << "=" << kv.value << std::endl;
          }
        }
      }
    });
  tf2_ros::StaticTransformBroadcaster static_tf(probe);
  tf2_ros::TransformBroadcaster truth_tf(probe);
  tf2_ros::Buffer tf_buffer(probe->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(component.get_node_base_interface());
  if (pipeline) {
    executor.add_node(livox_component->get_node_base_interface());
    executor.add_node(deskew_component->get_node_base_interface());
    executor.add_node(compact_component->get_node_base_interface());
  }
  executor.add_node(probe); executor.add_node(map_loader);
  std::exception_ptr spin_error;
  std::thread spinning([&] {
    try {executor.spin();} catch (...) {spin_error = std::current_exception();}
  });
  const auto cleanup = [&] {executor.cancel(); if (spinning.joinable()) spinning.join();};
  try {
    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (scans->get_subscription_count() == 0 || seeds->get_subscription_count() == 0 ||
      pose_sub->get_publisher_count() == 0 || aligned_sub->get_publisher_count() == 0 ||
      (pipeline && (motion->get_subscription_count() == 0 ||
        probe->count_publishers("extended") != 1 || probe->count_subscribers("extended") != 2 ||
        probe->count_publishers("rectified") != 1 || probe->count_subscribers("rectified") != 2 ||
        probe->count_publishers("compact") != 1 || probe->count_subscribers("compact") != 2))) {
      require(std::chrono::steady_clock::now() < deadline, "ROS discovery timeout");
      std::this_thread::sleep_for(20ms);
    }
    geometry_msgs::msg::TransformStamped identity;
    identity.header.frame_id = base_frame; identity.child_frame_id = scan_frame;
    identity.transform.rotation.w = 1.0;
    static_tf.sendTransform(identity);
    if (pipeline) {
      managed_transform_buffer::ManagedTransformBuffer preflight;
      auto transform = preflight.getTransform<Eigen::Matrix4f>(base_frame, scan_frame,
        probe->now(), rclcpp::Duration::from_seconds(2.0), probe->get_logger());
      require(transform.has_value() && transform->isApprox(Eigen::Matrix4f::Identity(), 1e-6F),
        "Managed TF readiness failed before starting the cloud chain");
    }
    require(tf_buffer.canTransform(base_frame, scan_frame, tf2::TimePointZero,
      tf2::durationFromSec(5.0)), "Static TF readiness timeout");
    require(activate->wait_for_service(10s), "Activation service discovery timeout");
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>(); request->data = true;
    auto future = activate->async_send_request(request);
    require(future.wait_for(10s) == std::future_status::ready, "Activation response timeout");
    require(future.get()->success, "NDT activation rejected");
    auto initial = make_pose(100.3F, 100.2F); initial.header.stamp = probe->now();
    seeds->publish(initial);
    require(observed.wait([&] {return observed.map && observed.map->width > 0;}),
      "NDT did not load the synthetic differential map");
    {
      std::lock_guard<std::mutex> lock(observed.mutex);
      require(observed.map->header.frame_id == "map" && observed.map->width == 30603,
        "Unexpected synthetic map frame or point count");
      std::cout << "NDT loaded map: " << observed.map->width << " points" << std::endl;
    }

    const auto run_position = [&](double x, double y) {
      builtin_interfaces::msg::Time stamp = static_cast<builtin_interfaces::msg::Time>(probe->now());
      stamp.nanosec = (stamp.nanosec / 256U) * 256U;
      auto old_pose = make_pose(static_cast<float>(x + 0.3), static_cast<float>(y + 0.2));
      auto new_pose = old_pose;
      const auto ros_stamp = rclcpp::Time(stamp);
      old_pose.header.stamp = static_cast<builtin_interfaces::msg::Time>(
        ros_stamp - rclcpp::Duration::from_seconds(0.05));
      new_pose.header.stamp = static_cast<builtin_interfaces::msg::Time>(
        ros_stamp + rclcpp::Duration::from_seconds(0.05));
      seeds->publish(old_pose); seeds->publish(new_pose);
      if (pipeline) {
        Twist first_twist, last_twist;
        first_twist.header.frame_id = base_frame; first_twist.header.stamp = stamp;
        last_twist.header.frame_id = base_frame;
        last_twist.header.stamp = static_cast<builtin_interfaces::msg::Time>(
          ros_stamp + rclcpp::Duration::from_seconds(0.05));
        first_twist.twist.twist.linear.x = 1.0;
        last_twist.twist.twist.linear.x = 1.0;
        motion->publish(first_twist); motion->publish(last_twist);
        require(seeds->wait_for_all_acked(2s) && motion->wait_for_all_acked(2s),
          "Pose seed or synthetic motion delivery timed out");
      }
      geometry_msgs::msg::TransformStamped truth;
      truth.header.stamp = stamp; truth.header.frame_id = "map"; truth.child_frame_id = base_frame;
      truth.transform.translation.x = x; truth.transform.translation.y = y;
      truth.transform.rotation.w = 1.0; truth_tf.sendTransform(truth);
      auto scan = make_default_sensor_pcd();
      pcl::PointCloud<pcl::PointXYZ> points;
      pcl::fromROSMsg(scan, points);
      for (auto & point : points) {
        point.x += static_cast<float>(100.0 - x);
        point.y += static_cast<float>(100.0 - y);
      }
      pcl::toROSMsg(points, scan);
      scan.header.frame_id = scan_frame; scan.header.stamp = stamp;
      Cloud raw_scan = pipeline ? livox_acquisition(scan) : scan;
      if (pipeline) {
        std::lock_guard<std::mutex> lock(observed.mutex);
        observed.extended.reset(); observed.rectified.reset(); observed.compact.reset();
      }
      // Bound delivery delay and allow both seed callbacks to finish before the scan.
      std::this_thread::sleep_for(100ms);
      scans->publish(raw_scan);
      require(observed.wait([&] {
        return observed.pose && observed.aligned &&
          observed.pose->header.stamp == raw_scan.header.stamp &&
          observed.aligned->header.stamp == raw_scan.header.stamp &&
          (!pipeline || (observed.extended && observed.rectified && observed.compact &&
            observed.extended->header.stamp == raw_scan.header.stamp &&
            observed.rectified->header.stamp == raw_scan.header.stamp &&
            observed.compact->header.stamp == raw_scan.header.stamp));
      }), "No converged NDT pose and aligned scan within 15 seconds");
      Pose::ConstSharedPtr result;
      Cloud::ConstSharedPtr aligned;
      Cloud::ConstSharedPtr extended, rectified, compact;
      {
        std::lock_guard<std::mutex> lock(observed.mutex);
        result = observed.pose; aligned = observed.aligned;
        extended = observed.extended; rectified = observed.rectified; compact = observed.compact;
      }
      if (pipeline) {
        verify_preprocessing(*extended, *rectified, *compact, raw_scan, scan);
      }
      const auto & p = result->pose.pose.position;
      const auto & q = result->pose.pose.orientation;
      require(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
        std::isfinite(q.w) && std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 1e-5,
        "Invalid orientation quaternion");
      const double error = std::hypot(std::hypot(p.x - x, p.y - y), p.z);
      const double angle = 2.0 * std::acos(std::min(1.0, std::abs(q.w)));
      std::cout << "ground_truth=(" << x << ',' << y << ",0), NDT=(" << p.x << ',' <<
        p.y << ',' << p.z << "), position_error=" << error <<
        " m, orientation_error=" << angle << " rad" << std::endl;
      require(result->header.frame_id == "map", "Wrong pose frame");
      require(std::isfinite(error) && error < 0.10, "Position error exceeds fixed 0.10 m bound");
      require(std::isfinite(angle) && angle < 0.02, "Orientation error exceeds fixed 0.02 rad bound");
      require(aligned->header.frame_id == "map" &&
        aligned->width == (pipeline ? compact->width : scan.width),
        "NDT aligned scan frame/count mismatch");
      require(tf_buffer.canTransform("map", estimate_frame, tf2::TimePointZero,
        tf2::durationFromSec(5.0)), "Estimated TF readiness timeout");
      const auto transform = tf_buffer.lookupTransform("map", estimate_frame, stamp,
        rclcpp::Duration::from_seconds(5.0));
      require(std::abs(transform.transform.translation.x - p.x) < 1e-5 &&
        std::abs(transform.transform.translation.y - p.y) < 1e-5 &&
        std::abs(transform.transform.translation.z - p.z) < 1e-5 &&
        std::abs(transform.transform.rotation.x - q.x) < 1e-5 &&
        std::abs(transform.transform.rotation.y - q.y) < 1e-5 &&
        std::abs(transform.transform.rotation.z - q.z) < 1e-5 &&
        std::abs(transform.transform.rotation.w - q.w) < 1e-5,
        "Estimated TF does not match published pose");
      return result->pose.pose.position;
    };
    const auto first = run_position(100.0, 100.0);
    const auto second = run_position(101.0, 100.5);
    require(std::abs((second.x - first.x) - 1.0) < 0.10 &&
      std::abs((second.y - first.y) - 0.5) < 0.10 &&
      std::abs(second.z - first.z) < 0.10, "Estimated displacement mismatch");
    std::cout << "PASS: two poses, controlled displacement, map, aligned scans and TF" <<
      (pipeline ? "; all three LiDAR preprocessing components executed" : "") << std::endl;
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(hold_seconds);
    bool alternate = false;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      std::this_thread::sleep_for(400ms);
      run_position(alternate ? 100.0 : 101.0, alternate ? 100.0 : 100.5);
      alternate = !alternate;
    }
    cleanup();
    if (spin_error) std::rethrow_exception(spin_error);
  } catch (...) {cleanup(); throw;}
  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {status = demonstration();}
  catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << std::endl; status = 1;
  }
  rclcpp::shutdown();
  return status;
}
