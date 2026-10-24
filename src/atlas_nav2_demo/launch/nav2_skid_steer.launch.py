"""Headless Nav2 plus ideal skid-steer simulation; no physical interfaces."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atlas_nav2_demo'))
    nav2_params = LaunchConfiguration('nav2_params')
    simulation_params = LaunchConfiguration('simulation_params')
    map_yaml = LaunchConfiguration('map')
    scenario = LaunchConfiguration('scenario')
    nav2_nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[nav2_params, {'yaml_filename': map_yaml}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav2_params],
             remappings=[('cmd_vel', '/atlas_nav2/cmd_vel_nav')]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[nav2_params],
             remappings=[('cmd_vel', '/atlas_nav2/cmd_vel_nav')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav2_params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[nav2_params]),
    ]
    simulation_nodes = [
        Node(package='atlas_nav2_demo', executable='safety_gate', name='safety_gate',
             output='screen', parameters=[simulation_params]),
        Node(package='atlas_nav2_demo', executable='wheel_mixer', name='wheel_mixer',
             output='screen', parameters=[simulation_params]),
        Node(package='atlas_nav2_demo', executable='skid_steer_simulator',
             name='skid_steer_simulator', output='screen',
             parameters=[simulation_params, {'scenario': scenario}]),
        Node(package='atlas_nav2_demo', executable='synthetic_laser',
             name='synthetic_laser', output='screen', parameters=[{'scenario': scenario}]),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('nav2_params', default_value=str(share / 'config/nav2_params.yaml')),
        DeclareLaunchArgument('simulation_params', default_value=str(share / 'config/simulation.yaml')),
        DeclareLaunchArgument('map', default_value=str(share / 'maps/synthetic_map.yaml')),
        DeclareLaunchArgument('scenario', default_value='open'),
        *simulation_nodes,
        *nav2_nodes,
    ])
