import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory('atlas_closed_loop_demo')
    core = '/home/marian/roam/references/autoware/src/core/autoware_core/planning'
    vehicle = os.path.join(share, 'config', 'test_vehicle.param.yaml')
    overrides = os.path.join(share, 'config', 'planning_overrides.param.yaml')
    results = LaunchConfiguration('results_dir')
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('atlas_map_integration_demo'), 'launch', 'campus_map.launch.py')),
        launch_arguments={'launch_rviz': 'false', 'results_dir': [results, '/map']}.items())
    fixture = Node(package='atlas_closed_loop_demo', executable='route_adapter',
                   name='route_adapter', namespace='/atlas_closed_loop', output='screen',
                   parameters=[{'results_dir': results}])
    planning = ComposableNodeContainer(
        name='planning_container', namespace='/atlas_closed_loop',
        package='rclcpp_components', executable='component_container', output='screen',
        condition=IfCondition(LaunchConfiguration('planning_enabled')),
        composable_node_descriptions=[
            ComposableNode(package='autoware_path_generator', plugin='autoware::path_generator::PathGenerator',
                           name='path_generator', namespace='/atlas_closed_loop',
                           parameters=[core + '/autoware_path_generator/config/path_generator.param.yaml',
                                       vehicle, overrides],
                           remappings=[('~/input/vector_map', '/atlas_map_demo/vector_map'),
                                       ('~/input/route', '/atlas_closed_loop/route'),
                                       ('~/input/odometry', '/atlas_closed_loop/feedback/odometry'),
                                       ('~/output/path', '/atlas_closed_loop/path_with_lane_id')]),
            ComposableNode(package='autoware_planning_topic_converter',
                           plugin='autoware::planning_topic_converter::PathToTrajectory',
                           name='path_to_trajectory_converter', namespace='/atlas_closed_loop',
                           parameters=[{'input_topic': '/atlas_closed_loop/path',
                                        'output_topic': '/atlas_closed_loop/raw_trajectory'}]),
        ])
    smoother = Node(package='autoware_velocity_smoother', executable='velocity_smoother_node',
                    name='velocity_smoother', namespace='/atlas_closed_loop', output='screen',
                    condition=IfCondition(LaunchConfiguration('planning_enabled')),
                    parameters=[core + '/autoware_velocity_smoother/config/default_common.param.yaml',
                                core + '/autoware_velocity_smoother/config/default_velocity_smoother.param.yaml',
                                core + '/autoware_velocity_smoother/config/Analytical.param.yaml',
                                vehicle, overrides, {'algorithm_type': 'Analytical'}],
                    remappings=[('/localization/kinematic_state', '/atlas_closed_loop/feedback/odometry'),
                                ('~/input/acceleration', '/atlas_closed_loop/feedback/acceleration'),
                                ('~/input/operation_mode_state', '/atlas_closed_loop/operation_mode'),
                                ('~/input/trajectory', '/atlas_closed_loop/raw_trajectory'),
                                ('~/output/trajectory', '/atlas_closed_loop/trajectory')])
    control_root = '/home/marian/roam/references/autoware/src/universe/autoware_universe/control/autoware_trajectory_follower_node'
    controller = Node(package='autoware_trajectory_follower_node', executable='controller_node_exe',
                      name='controller', namespace='/atlas_closed_loop', output='screen',
                      parameters=[control_root + '/param/trajectory_follower_node.param.yaml',
                                  control_root + '/param/lateral/mpc.param.yaml',
                                  control_root + '/param/longitudinal/pid.param.yaml', vehicle,
                                  os.path.join(share, 'config', 'controller_overrides.param.yaml'),
                                  {'stopped_acc': ParameterValue(PythonExpression(["'-0.2' if '", LaunchConfiguration('vehicle_model'), "' == 'IDEAL_STEER_ACC_GEARED' else '0.0'"]),value_type=float)}],
                      remappings=[('~/input/reference_trajectory', '/atlas_closed_loop/trajectory'),
                                  ('~/input/current_odometry', '/atlas_closed_loop/feedback/odometry'),
                                  ('~/input/current_accel', '/atlas_closed_loop/feedback/acceleration'),
                                  ('~/input/current_steering', '/atlas_closed_loop/feedback/steering'),
                                  ('~/input/current_operation_mode', '/atlas_closed_loop/operation_mode'),
                                  ('~/output/control_cmd', '/atlas_closed_loop/controller_raw/control_cmd')])
    simulator = ComposableNodeContainer(
        name='simulator_container', namespace='/atlas_closed_loop/sim',
        package='rclcpp_components', executable='component_container', output='screen',
        composable_node_descriptions=[ComposableNode(
            package='autoware_simple_planning_simulator',
            plugin='autoware::simulator::simple_planning_simulator::SimplePlanningSimulator',
            name='simple_planning_simulator', namespace='/atlas_closed_loop/sim',
            parameters=[os.path.join(share, 'config', 'simulator.param.yaml'),
                        {'vehicle_model_type': LaunchConfiguration('vehicle_model'),
                         'initial_engage_state': False}],
            remappings=[('input/ackermann_control_command', PythonExpression(["'/atlas_closed_loop/gated/control_cmd' if '",LaunchConfiguration('gated'),"' == 'true' else '/atlas_closed_loop/control_cmd'"])),
                        ('input/trajectory', '/atlas_closed_loop/trajectory')])])
    relay = Node(package='atlas_closed_loop_demo', executable='feedback_relay.py',
                 name='feedback_relay', namespace='/atlas_closed_loop', output='screen',
                 parameters=[{'release_delay_sec': 8.0, 'require_safety': ParameterValue(LaunchConfiguration('gated'),value_type=bool)}])
    gate = Node(package='autoware_vehicle_cmd_gate', executable='vehicle_cmd_gate_exe',
                name='vehicle_cmd_gate', namespace='/atlas_closed_loop/gate', output='screen',
                condition=IfCondition(LaunchConfiguration('gated')),
                parameters=[vehicle,os.path.join(share,'config','gate.param.yaml')],
                remappings=[('/localization/kinematic_state','/atlas_closed_loop/sim/output/odometry'),
                            ('input/acceleration','/atlas_closed_loop/feedback/acceleration'),
                            ('input/steering','/atlas_closed_loop/sim/output/steering'),
                            ('input/operation_mode','/atlas_closed_loop/operation_mode'),
                            ('input/auto/control_cmd','/atlas_closed_loop/control_cmd'),
                            ('input/auto/gear_cmd','/atlas_closed_loop/auto_gear'),
                            ('output/control_cmd','/atlas_closed_loop/gated/control_cmd'),
                            ('output/gear_cmd','/atlas_closed_loop/gated/gear_cmd')])
    watchdog = Node(package='atlas_closed_loop_demo', executable='feedback_watchdog.py',
                    name='feedback_watchdog', namespace='/atlas_closed_loop', output='screen',
                    condition=IfCondition(LaunchConfiguration('gated')))
    rviz = Node(package='rviz2', executable='rviz2', name='closed_loop_rviz',
                namespace='/atlas_closed_loop', output='screen',
                arguments=['-d', os.path.join(share, 'config', 'closed_loop.rviz')],
                condition=IfCondition(LaunchConfiguration('launch_rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('results_dir', default_value='/tmp/atlas_closed_loop_results/interactive'),
        DeclareLaunchArgument('gated',default_value='true'),
        DeclareLaunchArgument('vehicle_model',default_value='IDEAL_STEER_ACC_GEARED'),
        DeclareLaunchArgument('planning_enabled', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        GroupAction([map_launch], scoped=True), fixture, planning, smoother, simulator, relay, controller, gate, watchdog, rviz])
