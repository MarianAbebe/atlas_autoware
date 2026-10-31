"""Launch only the FAST-LIO odometry shadow validator; no estimator or control nodes."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atlas_lio_integration',
            executable='lio_shadow_adapter',
            name='atlas_lio_shadow_odometry',
            output='screen',
        ),
    ])
