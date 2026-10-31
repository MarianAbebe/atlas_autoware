"""Launch only the FAST-LIO pose-only shadow adapter."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atlas_lio_integration',
            executable='lio_pose_shadow_adapter',
            name='atlas_lio_shadow_pose',
            output='screen',
        ),
    ])
