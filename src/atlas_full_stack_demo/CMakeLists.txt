cmake_minimum_required(VERSION 3.14)
project(atlas_full_stack_demo)
find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/scene.py scripts/static_objects.py scripts/manual_start.py
  scripts/ego_pose_marker.py
  scripts/initialize_localization.py scripts/record_timing_diagnostics.sh
  scripts/scheduler_telemetry.py scripts/profile_scene.py
  scripts/run_lap_validation.py scripts/generate_five_lap_map.py
  scripts/run_scene_sensor_stress.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/lap_metrics.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
