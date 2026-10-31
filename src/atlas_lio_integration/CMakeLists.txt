cmake_minimum_required(VERSION 3.8)
project(atlas_lio_integration)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME} SKIP_COMPILE)
install(PROGRAMS
  scripts/lio_health
  scripts/evaluate_lio
  scripts/livox_adapter
  scripts/lio_pose_shadow_adapter
  scripts/lio_shadow_adapter
  scripts/record_pose_shadow
  scripts/analyze_pose_shadow
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
 find_package(ament_cmake_pytest REQUIRED)
 ament_add_pytest_test(lio_unit_tests test TIMEOUT 30)
endif()
ament_package()
