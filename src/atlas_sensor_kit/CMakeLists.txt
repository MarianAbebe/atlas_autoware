cmake_minimum_required(VERSION 3.8)
project(atlas_sensor_kit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_point_types QUIET)

add_library(pointcloud_conversion SHARED src/pointcloud_conversion.cpp)
target_compile_features(pointcloud_conversion PUBLIC cxx_std_17)
target_include_directories(pointcloud_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pointcloud_conversion sensor_msgs)
if(autoware_point_types_FOUND)
  ament_target_dependencies(pointcloud_conversion autoware_point_types)
  target_compile_definitions(pointcloud_conversion PRIVATE ATLAS_HAS_AUTOWARE_POINT_TYPES)
endif()

add_library(pointcloud_components SHARED src/pointcloud_components.cpp)
target_link_libraries(pointcloud_components pointcloud_conversion)
ament_target_dependencies(pointcloud_components rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(pointcloud_components
  PLUGIN "atlas_sensor_kit::LivoxToExtendedComponent"
  EXECUTABLE livox_to_extended_node)
rclcpp_components_register_node(pointcloud_components
  PLUGIN "atlas_sensor_kit::ExtendedToCompactComponent"
  EXECUTABLE extended_to_compact_node)

install(TARGETS pointcloud_conversion pointcloud_components
  EXPORT export_pointcloud_conversion
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_pointcloud_conversion HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs rclcpp rclcpp_components)
if(autoware_point_types_FOUND)
  ament_export_dependencies(autoware_point_types)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(class_loader REQUIRED)
  ament_add_gtest(test_pointcloud_conversion test/test_pointcloud_conversion.cpp)
  target_link_libraries(test_pointcloud_conversion pointcloud_conversion)
  ament_add_gtest(test_pointcloud_components test/test_pointcloud_components.cpp TIMEOUT 30)
  ament_target_dependencies(test_pointcloud_components rclcpp rclcpp_components sensor_msgs class_loader)
  target_compile_features(test_pointcloud_components PRIVATE cxx_std_17)
  target_compile_definitions(test_pointcloud_components PRIVATE
    ATLAS_COMPONENT_LIBRARY="$<TARGET_FILE:pointcloud_components>")
  add_dependencies(test_pointcloud_components pointcloud_components)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ros_log")
  set_tests_properties(test_pointcloud_components PROPERTIES
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=177;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log")
endif()

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
