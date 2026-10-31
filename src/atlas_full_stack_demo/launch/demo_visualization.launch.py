"""Start only the RViz demo view and a description-only RobotModel source."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('atlas_full_stack_demo'))
    description = (share / 'config' / 'demo_vehicle.urdf').read_text()
    default_rviz = str(share / 'config' / 'demo_resource_limited.rviz')

    description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='atlas_demo_robot_description',
        output='screen',
        parameters=[{'robot_description': description}],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='atlas_demo_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz configuration for the full-stack demonstration',
        ),
        description_publisher,
        rviz,
    ])
