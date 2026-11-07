"""Closed-loop synthetic Lanelet2 demonstration using the existing Atlas stack."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atlas_full_stack_demo'))
    maps = share / 'config' / 'five_lap_loop'
    full_stack = share / 'launch' / 'full_stack.launch.py'
    visualization = share / 'launch' / 'demo_visualization.launch.py'

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(full_stack)),
        launch_arguments={
            'obstacle': 'false',
            'request_engagement': 'false',
            'results_dir': LaunchConfiguration('results_dir'),
            'osm_path': str(maps / 'five_lap_loop.osm'),
            'pcd_path': str(maps / 'five_lap_loop.pcd'),
            'projector_path': str(maps / 'map_projector_info.yaml'),
            'route_start_lanelet': '100',
            'route_goal_lanelet': '108',
            'route_length_m': '63.375415291',
            'trajectory_repeats': LaunchConfiguration('laps'),
            'loop_transition_max_m': '8.5',
            'include_terminal_extension': 'true',
            'map_cloud_topic': '/atlas_map_demo/pointcloud_map',
            'lidar_max_range_m': '12.0',
            'launch_routing_probe': 'false',
            'ndt_num_threads': '2',
        }.items(),
    )
    operator_start = Node(
        package='atlas_full_stack_demo', executable='manual_start.py',
        name='five_lap_operator_start', output='screen',
    )
    ego_marker = Node(
        package='atlas_full_stack_demo', executable='ego_pose_marker.py',
        name='atlas_ego_pose_marker', output='screen',
        condition=IfCondition(LaunchConfiguration('launch_ego_marker')),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(visualization)),
        launch_arguments={'rviz_config': str(share / 'config' / 'five_lap.rviz')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('laps', default_value='1', description='1, 2, or 5 repeated laps in one planned trajectory'),
        DeclareLaunchArgument('results_dir', default_value='/tmp/atlas_five_lap_demo'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('launch_ego_marker', default_value='true',
                              description='Publish the visualization-only body marker from EKF odometry.'),
        stack,
        operator_start,
        ego_marker,
        TimerAction(period=10.0, actions=[rviz]),
    ])
