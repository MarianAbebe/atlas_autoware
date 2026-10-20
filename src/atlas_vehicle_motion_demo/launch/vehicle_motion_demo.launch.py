from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('atlas_vehicle_motion_demo')
    sim_parameters = os.path.join(share, 'config', 'simulator.param.yaml')
    rviz_config = os.path.join(share, 'config', 'vehicle_motion_demo.rviz')
    demo = Node(
        package='atlas_vehicle_motion_demo',
        executable='vehicle_motion_demo',
        name='bounded_motion_test',
        namespace='/atlas_vehicle_sim',
        output='screen',
        parameters=[{
            'timeout_sec': 15.0,
            'hold_after_test_sec': LaunchConfiguration('visualization_hold_sec'),
        }],
    )
    simulator = ComposableNodeContainer(
        name='simulator_container',
        namespace='/atlas_vehicle_sim',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='autoware_simple_planning_simulator',
                plugin='autoware::simulator::simple_planning_simulator::SimplePlanningSimulator',
                name='simple_planning_simulator',
                namespace='/atlas_vehicle_sim',
                parameters=[sim_parameters],
                extra_arguments=[{'use_intra_process_comms': False}],
            )
        ],
    )
    shutdown_on_test_exit = RegisterEventHandler(
        OnProcessExit(target_action=demo, on_exit=[EmitEvent(event=Shutdown())])
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='vehicle_motion_visualization',
        namespace='/atlas_vehicle_sim',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('visualization_hold_sec', default_value='0.0'),
        simulator,
        demo,
        rviz,
        shutdown_on_test_exit,
    ])
