cmake_minimum_required(VERSION 3.14)
project(atlas_lidar_perception_test)

find_package(ament_cmake REQUIRED)
foreach(dep ament_index_cpp atlas_distortion_corrector_test atlas_sensor_kit
    autoware_euclidean_cluster_object_detector autoware_perception_msgs class_loader
    geometry_msgs rclcpp rclcpp_components sensor_msgs tf2_ros visualization_msgs)
  find_package(${dep} REQUIRED)
endforeach()

add_executable(lidar_perception_demo test/lidar_perception_demo.cpp)
target_compile_features(lidar_perception_demo PRIVATE cxx_std_17)
target_compile_options(lidar_perception_demo PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(lidar_perception_demo ament_index_cpp class_loader geometry_msgs
  rclcpp rclcpp_components sensor_msgs tf2_ros visualization_msgs autoware_perception_msgs)
install(TARGETS lidar_perception_demo DESTINATION lib/${PROJECT_NAME})
install(FILES README.md config/lidar_perception.rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_test(NAME synthetic_lidar_obstacle_detection COMMAND lidar_perception_demo)
  set_tests_properties(synthetic_lidar_obstacle_detection PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/atlas_lidar_perception_demo_ros_log")
endif()

ament_package()
