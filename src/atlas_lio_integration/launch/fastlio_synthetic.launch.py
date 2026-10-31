"""Headless synthetic FAST-LIO2 pipeline with tracked Atlas configuration."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    config = LaunchConfiguration('fastlio_config').perform(context)
    if not os.path.isfile(config):
        raise RuntimeError(f'Required FAST-LIO2 configuration does not exist: {config}')
    return [
        Node(
            package='atlas_sensor_sim', executable='sensor_simulator',
            name='atlas_sensor_simulator',
            parameters=[{'lidar_rate_hz': 10.0, 'imu_rate_hz': 200.0}],
        ),
        Node(
            package='atlas_lio_integration', executable='livox_adapter',
            name='atlas_livox_adapter',
            parameters=[{'scan_period_sec': 0.1}],
        ),
        Node(
            package='fastlio2_ros2', executable='ros2_localizer', name='fastlio2',
            parameters=[{
                'config_path': config,
                'lidar_topic': '/atlas/lio/livox',
                'imu_topic': '/atlas/sensors/imu/data',
                'odom_topic': '/atlas/lio/odometry',
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
            }],
        ),
        Node(
            package='atlas_lio_integration', executable='lio_health',
            name='atlas_lio_health',
        ),
    ]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('atlas_lio_integration'),
        'config', 'fastlio_synthetic.yaml',
    )
    if not os.path.isfile(config):
        raise RuntimeError(f'Installed synthetic FAST-LIO2 config is missing: {config}')
    return LaunchDescription([
        DeclareLaunchArgument(
            'fastlio_config', default_value=config,
            description='FAST-LIO2 YAML config; default is the installed synthetic config',
        ),
        OpaqueFunction(function=_nodes),
    ])
