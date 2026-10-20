cmake_minimum_required(VERSION 3.14)
project(atlas_vehicle_motion_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(vehicle_motion_demo src/vehicle_motion_demo.cpp)
ament_target_dependencies(vehicle_motion_demo
  rclcpp autoware_control_msgs autoware_vehicle_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs tf2_ros
  visualization_msgs)
install(TARGETS vehicle_motion_demo DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
