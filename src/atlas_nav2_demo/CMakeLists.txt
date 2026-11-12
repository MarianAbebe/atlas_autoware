cmake_minimum_required(VERSION 3.8)
project(atlas_nav2_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(atlas_ackermann_controller SHARED src/ackermann_controller.cpp)
target_include_directories(atlas_ackermann_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(atlas_ackermann_controller
  geometry_msgs nav2_core nav2_costmap_2d nav_msgs pluginlib rclcpp
  rclcpp_lifecycle sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros)
pluginlib_export_plugin_description_file(nav2_core ackermann_controller_plugin.xml)
install(TARGETS atlas_ackermann_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/safety_gate
  scripts/ackermann_adapter
  scripts/ackermann_simulator
  scripts/ackermann_showcase_joints
  scripts/ackermann_showcase_course
  scripts/skid_steer_simulator
  scripts/synthetic_laser
  scripts/validate_closed_loop.py
  scripts/validate_safety_gate.py
  scripts/record_ackermann_navigation.py
  scripts/run_ackermann_acceptance.py
  scripts/validate_ackermann_faults.py
  scripts/validate_ackermann_controller.py
  scripts/wheel_mixer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config maps behavior_trees urdf rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ackermann_controller_math test/test_ackermann_controller_math.cpp)
  target_include_directories(test_ackermann_controller_math PRIVATE include)
  ament_add_pytest_test(nav2_demo_unit_tests test TIMEOUT 30)
endif()

ament_export_include_directories(include)
ament_export_libraries(atlas_ackermann_controller)
ament_export_dependencies(nav2_core nav2_costmap_2d pluginlib)
ament_package()
