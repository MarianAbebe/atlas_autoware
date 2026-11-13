import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory('atlas_trajectory_generation_demo')
    autoware_root = os.environ.get('ATLAS_AUTOWARE_ROOT')
    if not autoware_root:
        raise RuntimeError('Set ATLAS_AUTOWARE_ROOT to the pinned Autoware checkout before launch')
    core = os.path.join(autoware_root, 'src/core/autoware_core/planning')
    vehicle = os.path.join(get_package_share_directory('atlas_vehicle_interface'),
                           'config', 'ackermann_vehicle.param.yaml')
    overrides = os.path.join(share, 'config', 'planning_overrides.param.yaml')
    results = LaunchConfiguration('results_dir')
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('atlas_map_integration_demo'), 'launch', 'campus_map.launch.py')),
        launch_arguments={'launch_rviz': 'false', 'results_dir': [results, '/map']}.items())
    fixture = Node(package='atlas_trajectory_generation_demo', executable='trajectory_fixture',
                   name='trajectory_fixture', namespace='/atlas_trajectory_demo', output='screen',
                   parameters=[{'results_dir': results, 'fixture_speed_mps': 0.2}])
    planning = ComposableNodeContainer(
        name='planning_container', namespace='/atlas_trajectory_demo',
        package='rclcpp_components', executable='component_container', output='screen',
        condition=IfCondition(LaunchConfiguration('planning_enabled')),
        composable_node_descriptions=[
            ComposableNode(package='autoware_path_generator', plugin='autoware::path_generator::PathGenerator',
                           name='path_generator', namespace='/atlas_trajectory_demo',
                           parameters=[core + '/autoware_path_generator/config/path_generator.param.yaml',
                                       vehicle, overrides],
                           remappings=[('~/input/vector_map', '/atlas_map_demo/vector_map'),
                                       ('~/input/route', '/atlas_trajectory_demo/route'),
                                       ('~/input/odometry', '/atlas_trajectory_demo/synthetic/odometry'),
                                       ('~/output/path', '/atlas_trajectory_demo/path_with_lane_id')]),
            ComposableNode(package='autoware_planning_topic_converter',
                           plugin='autoware::planning_topic_converter::PathToTrajectory',
                           name='path_to_trajectory_converter', namespace='/atlas_trajectory_demo',
                           parameters=[{'input_topic': '/atlas_trajectory_demo/path',
                                        'output_topic': '/atlas_trajectory_demo/raw_trajectory'}]),
        ])
    smoother = Node(package='autoware_velocity_smoother', executable='velocity_smoother_node',
                    name='velocity_smoother', namespace='/atlas_trajectory_demo', output='screen',
                    condition=IfCondition(LaunchConfiguration('planning_enabled')),
                    parameters=[core + '/autoware_velocity_smoother/config/default_common.param.yaml',
                                core + '/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml',
                                core + '/autoware_velocity_smoother/config/Analytical.param.yaml',
                                vehicle, overrides, {'algorithm_type': 'Analytical'}],
                    remappings=[('/localization/kinematic_state', '/atlas_trajectory_demo/synthetic/odometry'),
                                ('~/input/acceleration', '/atlas_trajectory_demo/synthetic/acceleration'),
                                ('~/input/operation_mode_state', '/atlas_trajectory_demo/synthetic/operation_mode'),
                                ('~/input/trajectory', '/atlas_trajectory_demo/raw_trajectory'),
                                ('~/output/trajectory', '/atlas_trajectory_demo/trajectory')])
    rviz = Node(package='rviz2', executable='rviz2', name='trajectory_rviz',
                namespace='/atlas_trajectory_demo', output='screen',
                arguments=['-d', os.path.join(share, 'config', 'trajectory_demo.rviz')],
                condition=IfCondition(LaunchConfiguration('launch_rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('results_dir', default_value='/tmp/atlas_trajectory_results/interactive'),
        DeclareLaunchArgument('planning_enabled', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        GroupAction([map_launch], scoped=True), fixture, planning, smoother, rviz])
