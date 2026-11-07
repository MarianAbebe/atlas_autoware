"""Explicit experimental integration; actual campus vector map, synthetic LiDAR map."""
import importlib.util
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ns='/atlas_loc';old=Path(share('atlas_closed_loop_demo'))/'config'
    path=Path(share('atlas_localization_closed_loop_demo'))/'launch/localization_loop.launch.py'
    spec=importlib.util.spec_from_file_location('localization_composition',path);loc=importlib.util.module_from_spec(spec);spec.loader.exec_module(loc)
    base=loc.generate_launch_description(include_fixture=False,ndt_points=ns+'/lidar/compact',smoother_input=ns+'/obstacle_trajectory',watchdog_outputs=[('gate/input/external_emergency_stop_heartbeat',ns+'/base_safety_heartbeat'),('safety_status',ns+'/base_safety_status')],ndt_num_threads=LaunchConfiguration('ndt_num_threads'))
    configured_autoware_root = os.environ.get('ATLAS_AUTOWARE_ROOT')
    if not configured_autoware_root:
        raise RuntimeError('Set ATLAS_AUTOWARE_ROOT to the pinned Autoware 1.9.0 checkout before launch')
    autoware_root=Path(configured_autoware_root).expanduser().resolve()
    core=autoware_root/'src/core/autoware_core';vehicle=str(old/'test_vehicle.param.yaml')
    perception_share=Path(share('atlas_perception_planning_demo'))
    preprocessing=ComposableNodeContainer(name='perception_container',namespace=ns,package='rclcpp_components',executable='component_container',output='screen',composable_node_descriptions=[
        ComposableNode(package='atlas_sensor_kit',plugin='atlas_sensor_kit::LivoxToExtendedComponent',name='livox_converter',namespace=ns,remappings=[('~/input/pointcloud',ns+'/lidar/raw'),('~/output/pointcloud',ns+'/lidar/extended')]),
        ComposableNode(package='atlas_distortion_corrector_test',plugin='autoware::pointcloud_preprocessor::DistortionCorrectorComponent',name='distortion_corrector',namespace=ns,parameters=[{'base_frame':'base_link','use_imu':False,'use_3d_distortion_correction':False,'update_azimuth_and_distance':False,'processing_time_threshold_sec':.05,'timestamp_mismatch_fraction_threshold':.01}],remappings=[('~/input/pointcloud',ns+'/lidar/extended'),('~/input/twist',ns+'/sensors/twist'),('~/output/pointcloud',ns+'/lidar/deskewed')]),
        ComposableNode(package='atlas_sensor_kit',plugin='atlas_sensor_kit::ExtendedToCompactComponent',name='compact_converter',namespace=ns,remappings=[('~/input/pointcloud',ns+'/lidar/deskewed'),('~/output/pointcloud',ns+'/lidar/compact')]),
        ComposableNode(package='autoware_ground_filter',plugin='autoware::ground_filter::GroundFilterComponent',name='ground_filter',namespace=ns,parameters=[str(core/'perception/autoware_ground_filter/config/ground_filter.param.yaml'),vehicle,str(perception_share/'config/ground_overrides.param.yaml')],remappings=[('input',ns+'/lidar/compact'),('output',ns+'/obstacles/pointcloud')]),
        ComposableNode(package='autoware_euclidean_cluster_object_detector',plugin='autoware::euclidean_cluster::EuclideanClusterNode',name='euclidean_cluster',namespace=ns,parameters=[str(core/'perception/autoware_euclidean_cluster_object_detector/config/euclidean_cluster.param.yaml')],remappings=[('input',ns+'/obstacles/pointcloud'),('output',ns+'/obstacles/detected_objects'),('debug/clusters',ns+'/clusters')])])
    planner_root=core/'planning';planner=Node(package='autoware_motion_velocity_planner',executable='autoware_motion_velocity_planner_node',name='motion_velocity_planner',namespace=ns,output='screen',parameters=[str(planner_root/p) for p in ['autoware_core_planning/config/common.param.yaml','autoware_core_planning/config/nearest_search.param.yaml','autoware_velocity_smoother/config/default_common.param.yaml','autoware_velocity_smoother/config/default_velocity_smoother.param.yaml','autoware_velocity_smoother/config/Analytical.param.yaml','motion_velocity_planner/autoware_motion_velocity_planner/config/motion_velocity_planner.param.yaml','motion_velocity_planner/autoware_motion_velocity_obstacle_stop_module/config/obstacle_stop.param.yaml']]+[vehicle,str(old/'planning_overrides.param.yaml'),str(Path(share('atlas_obstacle_stopping_demo'))/'config/obstacle_stop.param.yaml'),str(Path(share('atlas_full_stack_demo'))/'config/object_only_stop.param.yaml')],remappings=[('~/input/trajectory',ns+'/raw_trajectory'),('~/input/vector_map','/atlas_map_demo/vector_map'),('~/input/vehicle_odometry',ns+'/ekf/ekf_odom'),('~/input/accel',ns+'/sensors/acceleration'),('~/input/dynamic_objects',ns+'/obstacles/predicted_objects'),('~/input/no_ground_pointcloud',ns+'/obstacles/pointcloud'),('~/output/trajectory',ns+'/obstacle_trajectory')])
    map_dir=os.environ.get('ATLAS_MAP_DIR', str(Path.home()/'atlas_assets'))
    return LaunchDescription([
        DeclareLaunchArgument('obstacle',default_value='true'),
        DeclareLaunchArgument('ndt_num_threads',default_value='1',description='NDT worker threads; five-lap scenario selects 2 for sustained synthetic scan processing.'),
        DeclareLaunchArgument('request_engagement',default_value='true'),
        DeclareLaunchArgument('results_dir',default_value='/tmp/atlas_full_stack_manual'),
        DeclareLaunchArgument('osm_path',default_value=str(Path(map_dir)/'finalMap_repaired.osm')),
        DeclareLaunchArgument('pcd_path',default_value=str(Path(map_dir)/'pointcloud_map_georeferenced.pcd')),
        DeclareLaunchArgument('projector_path',default_value=str(Path(map_dir)/'map_projector_info.yaml')),
        DeclareLaunchArgument('route_start_lanelet',default_value='38184'),
        DeclareLaunchArgument('route_goal_lanelet',default_value='23892'),
        DeclareLaunchArgument('route_length_m',default_value='16.0'),
        DeclareLaunchArgument('trajectory_repeats',default_value='1'),
        DeclareLaunchArgument('loop_transition_max_m',default_value='6.5'),
        DeclareLaunchArgument('include_terminal_extension',default_value='false'),
        DeclareLaunchArgument('map_cloud_topic',default_value=''),
        DeclareLaunchArgument('lidar_max_range_m',default_value='0.0'),
        DeclareLaunchArgument('launch_routing_probe',default_value='true'),
    ]+list(base.entities)+[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('atlas_map_integration_demo'))/'launch/campus_map.launch.py')),launch_arguments={'launch_rviz':'false','launch_routing_probe':LaunchConfiguration('launch_routing_probe'),'results_dir':LaunchConfiguration('results_dir'),'osm_path':LaunchConfiguration('osm_path'),'pcd_path':LaunchConfiguration('pcd_path'),'projector_path':LaunchConfiguration('projector_path')}.items()),
        Node(package='atlas_closed_loop_demo',executable='route_adapter',name='route_adapter',namespace=ns,parameters=[{'results_dir':LaunchConfiguration('results_dir'),'start_lanelet_id':ParameterValue(LaunchConfiguration('route_start_lanelet'),value_type=int),'goal_lanelet_id':ParameterValue(LaunchConfiguration('route_goal_lanelet'),value_type=int)}],output='screen'),
        Node(package='atlas_full_stack_demo',executable='scene.py',parameters=[{'obstacle':ParameterValue(LaunchConfiguration('obstacle'),value_type=bool),'request_engagement':ParameterValue(LaunchConfiguration('request_engagement'),value_type=bool),'route_length_m':ParameterValue(LaunchConfiguration('route_length_m'),value_type=float),'trajectory_repeats':ParameterValue(LaunchConfiguration('trajectory_repeats'),value_type=int),'loop_transition_max_m':ParameterValue(LaunchConfiguration('loop_transition_max_m'),value_type=float),'include_terminal_extension':ParameterValue(LaunchConfiguration('include_terminal_extension'),value_type=bool),'map_cloud_topic':LaunchConfiguration('map_cloud_topic'),'lidar_max_range_m':ParameterValue(LaunchConfiguration('lidar_max_range_m'),value_type=float)}],output='screen'),
        Node(package='atlas_full_stack_demo',executable='static_objects.py',output='screen'),
        Node(package='atlas_perception_planning_demo',executable='perception_guard.py',name='obstacle_guard',namespace=ns,output='screen'),preprocessing,planner])
