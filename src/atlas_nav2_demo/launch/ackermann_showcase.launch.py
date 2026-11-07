"""RViz-only Atlas showcase; includes the authoritative deterministic Nav2 graph."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
 s=Path(get_package_share_directory('atlas_nav2_demo')); scenario=LaunchConfiguration('scenario')
 return LaunchDescription([
  DeclareLaunchArgument('scenario',default_value='open'),DeclareLaunchArgument('rviz',default_value='true'),
  IncludeLaunchDescription(PythonLaunchDescriptionSource(str(s/'launch/nav2_ackermann.launch.py')),launch_arguments={'scenario':scenario}.items()),
  Node(package='robot_state_publisher',executable='robot_state_publisher',name='atlas_showcase_description',parameters=[{'robot_description':(s/'urdf/atlas_rover_showcase.urdf').read_text()}],remappings=[('/joint_states','/atlas_ackermann/showcase_joint_states')]),
  Node(package='atlas_nav2_demo',executable='ackermann_showcase_joints',name='ackermann_showcase_joints'),
  Node(package='atlas_nav2_demo',executable='ackermann_showcase_course',name='ackermann_showcase_course',parameters=[{'scenario':scenario}]),
  Node(package='rviz2',executable='rviz2',name='atlas_ackermann_rviz',arguments=['-d',str(s/'rviz/ackermann_showcase.rviz')],condition=IfCondition(LaunchConfiguration('rviz'))),
 ])
