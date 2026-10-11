cmake_minimum_required(VERSION 3.14)
project(atlas_distortion_corrector_test LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
set(ATLAS_AUTOWARE_REFERENCE "$ENV{HOME}/roam/references/autoware" CACHE PATH
  "Read-only pinned Autoware reference root")
set(pre "${ATLAS_AUTOWARE_REFERENCE}/src/universe/autoware_universe/sensing/autoware_pointcloud_preprocessor")
set(tf "${ATLAS_AUTOWARE_REFERENCE}/src/universe/external/managed_transform_buffer/managed_transform_buffer")
set(utils "${ATLAS_AUTOWARE_REFERENCE}/src/core/autoware_utils")
set(types "${ATLAS_AUTOWARE_REFERENCE}/src/core/autoware_core/common/autoware_point_types")
set(upstream_sources
  "${pre}/src/distortion_corrector/distortion_corrector.cpp"
  "${pre}/src/distortion_corrector/distortion_corrector_node.cpp"
  "${pre}/src/utility/memory.cpp"
  "${tf}/src/managed_transform_buffer.cpp"
  "${tf}/src/managed_transform_buffer_provider.cpp"
  "${utils}/autoware_utils_math/src/trigonometry.cpp"
  "${utils}/autoware_utils_math/src/sin_table.cpp")
foreach(source IN LISTS upstream_sources)
  if(NOT EXISTS "${source}")
    message(FATAL_ERROR "Pinned upstream source missing: ${source}")
  endif()
endforeach()
if(BUILD_TESTING)
  set(ros_deps rclcpp rclcpp_components class_loader geometry_msgs sensor_msgs
    diagnostic_msgs autoware_internal_debug_msgs tf2 tf2_ros tf2_eigen
    tf2_geometry_msgs tf2_msgs point_cloud_msg_wrapper pcl_ros)
  foreach(dep IN LISTS ros_deps)
    find_package(${dep} REQUIRED)
  endforeach()
  find_package(Sophus REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(atlas_sensor_kit REQUIRED)
  # Discover the installed converter plugin, but never link it into the test.
  get_filename_component(sensor_prefix "${atlas_sensor_kit_DIR}/../../.." ABSOLUTE)
  set(converter_library "${sensor_prefix}/lib/libpointcloud_components.so")
  if(NOT EXISTS "${converter_library}")
    message(FATAL_ERROR "Build atlas_sensor_kit first; plugin missing: ${converter_library}")
  endif()
  # Keep registration in a plugin loaded only by class_loader. The test links
  # support for managed-TF preflight, sharing the component's singleton provider.
  set(component_source "${pre}/src/distortion_corrector/distortion_corrector_node.cpp")
  list(REMOVE_ITEM upstream_sources "${component_source}")
  add_library(pinned_distortion_support SHARED ${upstream_sources})
  target_compile_features(pinned_distortion_support PUBLIC cxx_std_17)
  target_include_directories(pinned_distortion_support PUBLIC
    "${pre}/include" "${tf}/include" "${types}/include" ${PCL_INCLUDE_DIRS})
  foreach(part autoware_utils autoware_utils_debug autoware_utils_diagnostics
      autoware_utils_rclcpp autoware_utils_system autoware_utils_math)
    target_include_directories(pinned_distortion_support PUBLIC "${utils}/${part}/include")
  endforeach()
  ament_target_dependencies(pinned_distortion_support ${ros_deps})
  target_link_libraries(pinned_distortion_support Eigen3::Eigen Sophus::Sophus ${PCL_LIBRARIES})
  add_library(pinned_distortion_corrector SHARED "${component_source}")
  target_link_libraries(pinned_distortion_corrector pinned_distortion_support)
  rclcpp_components_register_nodes(pinned_distortion_corrector
    "autoware::pointcloud_preprocessor::DistortionCorrectorComponent")
  ament_add_gtest_executable(test_distortion_corrector_component
    test/test_distortion_corrector_component.cpp)
  target_link_libraries(test_distortion_corrector_component pinned_distortion_support)
  add_dependencies(test_distortion_corrector_component pinned_distortion_corrector)
  target_compile_definitions(test_distortion_corrector_component PRIVATE
    ATLAS_COMPONENT_LIBRARY="$<TARGET_FILE:pinned_distortion_corrector>"
    ATLAS_CONVERTER_LIBRARY="${converter_library}")
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ros_log")
  foreach(scenario Forward Zero Missing Stale)
    set(result "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/deskew_${scenario}.gtest.xml")
    ament_add_test(deskew_${scenario}
      COMMAND $<TARGET_FILE:test_distortion_corrector_component>
        "--gtest_filter=DistortionCorrector.${scenario}" "--gtest_output=xml:${result}"
      RESULT_FILE "${result}"
      OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/deskew_${scenario}.txt"
      TIMEOUT 30)
    set_tests_properties(deskew_${scenario} PROPERTIES RUN_SERIAL TRUE
      ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=178;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log")
  endforeach()
  foreach(scenario Forward Zero Missing Stale)
    set(result "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/chain_${scenario}.gtest.xml")
    ament_add_test(chain_${scenario}
      COMMAND $<TARGET_FILE:test_distortion_corrector_component>
        "--gtest_filter=EndToEnd.${scenario}" "--gtest_output=xml:${result}"
      RESULT_FILE "${result}"
      OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/chain_${scenario}.txt"
      TIMEOUT 30)
    set_tests_properties(chain_${scenario} PROPERTIES RUN_SERIAL TRUE
      ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=178;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_log")
  endforeach()
  install(TARGETS pinned_distortion_corrector pinned_distortion_support LIBRARY DESTINATION lib)
endif()
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
