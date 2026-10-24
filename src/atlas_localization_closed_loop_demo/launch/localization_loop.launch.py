"""Short synthetic NDT -> EKF -> pinned MPC loop; no campus/obstacle launch."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description(include_fixture=True, ndt_points=None, smoother_input=None, watchdog_outputs=None):
    ns = '/atlas_loc'
    old = Path(share('atlas_closed_loop_demo')) / 'config'
    control = Path(share('autoware_trajectory_follower_node')) / 'param'
    vehicle = str(old / 'test_vehicle.param.yaml')
    simulator = ComposableNodeContainer(
        name='simulator_container', namespace=ns+'/sim',
        package='rclcpp_components', executable='component_container', output='screen',
        composable_node_descriptions=[ComposableNode(
            package='autoware_simple_planning_simulator',
            plugin='autoware::simulator::simple_planning_simulator::SimplePlanningSimulator',
            name='simple_planning_simulator', namespace=ns+'/sim',
            parameters=[str(old/'simulator.param.yaml'), {
                'vehicle_model_type': 'IDEAL_STEER_ACC_GEARED',
                'simulated_frame_id': 'hidden_truth_base', 'initial_engage_state': False}],
            remappings=[('input/ackermann_control_command', ns+'/gated/control_cmd'),
                        ('input/trajectory', ns+'/trajectory')])])
    ndt = Node(package='autoware_ndt_scan_matcher', executable='autoware_ndt_scan_matcher_node',
        name='ndt_scan_matcher', namespace=ns+'/ndt', output='screen',
        parameters=[share('autoware_ndt_scan_matcher')+'/config/ndt_scan_matcher.param.yaml',
                    {'ndt.num_threads': 1, 'dynamic_map_loading.publish_loaded_map': True}],
        remappings=[('pcd_loader_service', '/pcd_loader_service'),
                    ('points_raw', ndt_points or ns+'/sensors/points'),
                    ('ekf_pose_with_covariance', ns+'/ekf/ekf_pose_with_covariance')])
    ekf = Node(package='autoware_ekf_localizer', executable='autoware_ekf_localizer_node',
        name='ekf_localizer', namespace=ns+'/ekf', output='screen',
        parameters=[share('autoware_ekf_localizer')+'/config/ekf_localizer.param.yaml'],
        remappings=[('initialpose', ns+'/bootstrap'),
                    ('in_pose_with_covariance', ns+'/ndt/ndt_pose_with_covariance'),
                    ('in_twist_with_covariance', ns+'/sensors/twist')])
    controller = Node(package='autoware_trajectory_follower_node', executable='controller_node_exe',
        name='controller', namespace=ns, output='screen',
        parameters=[str(control/'trajectory_follower_node.param.yaml'),
                    str(control/'lateral/mpc.param.yaml'), str(control/'longitudinal/pid.param.yaml'),
                    vehicle, str(old/'controller_overrides.param.yaml'), {'stopped_acc': -.2}],
        remappings=[('~/input/reference_trajectory', ns+'/trajectory'),
                    ('~/input/current_odometry', ns+'/ekf/ekf_odom'),
                    ('~/input/current_accel', ns+'/sensors/acceleration'),
                    ('~/input/current_steering', ns+'/sim/output/steering'),
                    ('~/input/current_operation_mode', ns+'/operation_mode'),
                    ('~/output/control_cmd', ns+'/control_cmd')])
    smoother_root = Path(share('autoware_velocity_smoother')) / 'config'
    smoother = Node(package='autoware_velocity_smoother', executable='velocity_smoother_node',
        name='velocity_smoother', namespace=ns, output='screen',
        parameters=[str(smoother_root/'default_common.param.yaml'),
                    str(smoother_root/'default_velocity_smoother.param.yaml'),
                    str(smoother_root/'Analytical.param.yaml'), vehicle,
                    str(old/'planning_overrides.param.yaml'),
                    {'algorithm_type': 'Analytical', 'max_velocity': 1., 'max_vel': 1.}],
        remappings=[('/localization/kinematic_state', ns+'/ekf/ekf_odom'),
                    ('~/input/acceleration', ns+'/sensors/acceleration'),
                    ('~/input/operation_mode_state', ns+'/operation_mode'),
                    ('~/input/trajectory', smoother_input or ns+'/raw_trajectory'),
                    ('~/output/trajectory', ns+'/trajectory')])
    gate = Node(package='autoware_vehicle_cmd_gate', executable='vehicle_cmd_gate_exe',
        name='vehicle_cmd_gate', namespace=ns+'/gate', output='screen',
        parameters=[vehicle, str(old/'gate.param.yaml')],
        remappings=[('/localization/kinematic_state', ns+'/ekf/ekf_odom'),
                    ('input/acceleration', ns+'/sensors/acceleration'),
                    ('input/steering', ns+'/sim/output/steering'),
                    ('input/operation_mode', ns+'/operation_mode'),
                    ('input/auto/control_cmd', ns+'/control_cmd'),
                    ('input/auto/gear_cmd', ns+'/auto_gear'),
                    ('output/control_cmd', ns+'/gated/control_cmd')])
    watchdog = Node(package='atlas_localization_closed_loop_demo', executable='localization_watchdog.py',
        name='localization_watchdog', namespace=ns, output='screen',
        remappings=(watchdog_outputs or []) + [('feedback/odometry', ns+'/ekf/ekf_odom'),
                    ('feedback/acceleration', ns+'/sensors/acceleration'),
                    ('feedback/steering', ns+'/sim/output/steering')])
    fixture = Node(package='atlas_localization_closed_loop_demo', executable='localization_fixture',
                   output='screen')
    return LaunchDescription([simulator, ndt, ekf] + ([fixture] if include_fixture else []) + [smoother, controller, gate, watchdog])
