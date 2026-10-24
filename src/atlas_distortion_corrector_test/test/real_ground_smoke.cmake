# Optional real-data-only plane-filter probe. Compiles unchanged pinned sources.
find_package(message_filters REQUIRED)
find_package(autoware_internal_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
set(ground "${ATLAS_AUTOWARE_REFERENCE}/src/universe/autoware_universe/perception/autoware_ground_segmentation")
add_library(pinned_ransac_ground SHARED
  "${ground}/src/ransac_ground_filter/node.cpp" "${pre}/src/filter.cpp")
target_link_libraries(pinned_ransac_ground pinned_distortion_support ${PCL_LIBRARIES} Boost::thread)
ament_target_dependencies(pinned_ransac_ground message_filters pcl_msgs autoware_internal_msgs)
add_executable(real_ground_smoke test/real_ground_smoke.cpp)
ament_target_dependencies(real_ground_smoke rclcpp rclcpp_components sensor_msgs class_loader)
target_compile_features(real_ground_smoke PRIVATE cxx_std_17)
target_compile_definitions(real_ground_smoke PRIVATE
  ATLAS_GROUND_LIBRARY="$<TARGET_FILE:pinned_ransac_ground>"
  ATLAS_GROUND_PARAMS="${ground}/config/ransac_ground_filter.param.yaml")
add_dependencies(real_ground_smoke pinned_ransac_ground)
