"""Headless external FAST-LIO2 wiring. Requires pinned fastlio2_ros2 dependency."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
def generate_launch_description():
 return LaunchDescription([
  DeclareLaunchArgument(
   'fastlio_config',
   default_value=PathJoinSubstitution([
    EnvironmentVariable('HOME'),
    'atlas_fast_lio', 'fast_lio_core', 'config', 'mid360.yaml'
   ]),
   description='Absolute path to FAST-LIO2 YAML configuration'
  ),
  Node(package='atlas_sensor_sim',executable='sensor_simulator',name='atlas_sensor_simulator',parameters=[{'lidar_rate_hz':10.0,'imu_rate_hz':200.0}]),
  Node(package='atlas_lio_integration',executable='livox_adapter',name='atlas_livox_adapter'),
  Node(package='fastlio2_ros2',executable='ros2_localizer',name='fastlio2',parameters=[{'config_path':LaunchConfiguration('fastlio_config'),'lidar_topic':'/atlas/lio/livox','imu_topic':'/atlas/sensors/imu/data','odom_topic':'/atlas/lio/odometry','frame_id':'odom','child_frame_id':'base_link'}]),
  Node(package='atlas_lio_integration',executable='lio_health',name='atlas_lio_health')])
