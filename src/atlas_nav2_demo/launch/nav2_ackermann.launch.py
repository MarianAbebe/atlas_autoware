"""Isolated Ackermann Nav2 graph; never launch with Autoware control."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
 s=Path(get_package_share_directory('atlas_nav2_demo'));params=LaunchConfiguration('nav2_params');sim=LaunchConfiguration('simulation_params');scenario=LaunchConfiguration('scenario');map_yaml=LaunchConfiguration('map');initial_x=LaunchConfiguration('initial_x_m');initial_y=LaunchConfiguration('initial_y_m');ackermann_tree=str(s/'behavior_trees/ackermann_navigate_to_pose.xml')
 return LaunchDescription([
  DeclareLaunchArgument('nav2_params',default_value=str(s/'config/ackermann_nav2_params.yaml')),
  DeclareLaunchArgument('simulation_params',default_value=str(s/'config/ackermann_simulation.yaml')),
  DeclareLaunchArgument('map',default_value=str(s/'maps/synthetic_map.yaml')),
  DeclareLaunchArgument('scenario',default_value='open'),
  DeclareLaunchArgument('initial_x_m',default_value='-1.0'),
  DeclareLaunchArgument('initial_y_m',default_value='0.0'),
  Node(package='atlas_nav2_demo',executable='safety_gate',name='safety_gate',parameters=[sim],remappings=[('/atlas_nav2/cmd_vel_nav','/atlas_ackermann/cmd_vel_nav'),('/atlas_nav2/cmd_vel_safe','/atlas_ackermann/cmd_vel_safe'),('/atlas_nav2/odom','/atlas_ackermann/odom')]),
  Node(package='atlas_nav2_demo',executable='ackermann_adapter',name='ackermann_adapter',parameters=[sim]),
  Node(package='atlas_nav2_demo',executable='ackermann_simulator',name='ackermann_simulator',parameters=[sim,{'scenario':scenario,'initial_x_m':initial_x,'initial_y_m':initial_y}]),
  Node(package='atlas_nav2_demo',executable='synthetic_laser',name='ackermann_synthetic_laser',parameters=[{'scenario':scenario,'odom_topic':'/atlas_ackermann/odom','scan_topic':'/atlas_ackermann/scan'}]),
  Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[params,{'yaml_filename':map_yaml}]),
  Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[params]),
  Node(package='nav2_controller',executable='controller_server',name='controller_server',parameters=[params],remappings=[('cmd_vel','/atlas_ackermann/cmd_vel_nav')]),
  Node(package='nav2_behaviors',executable='behavior_server',name='behavior_server',parameters=[params]),
  Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',parameters=[params,{'default_nav_to_pose_bt_xml':ackermann_tree}]),
  Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',parameters=[params]),
 ])
