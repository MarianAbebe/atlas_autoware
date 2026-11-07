cmake_minimum_required(VERSION 3.8)
project(atlas_nav2_demo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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
  scripts/wheel_mixer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config maps behavior_trees urdf rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(nav2_demo_unit_tests test TIMEOUT 30)
endif()

ament_package()
