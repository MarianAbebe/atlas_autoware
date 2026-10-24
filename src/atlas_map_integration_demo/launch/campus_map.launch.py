import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    osm = LaunchConfiguration('osm_path')
    pcd = LaunchConfiguration('pcd_path')
    projector = LaunchConfiguration('projector_path')
    share = get_package_share_directory('atlas_map_integration_demo')
    container = ComposableNodeContainer(
        name='map_container', namespace='/atlas_map_demo',
        package='rclcpp_components', executable='component_container', output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='autoware_map_projection_loader',
                plugin='autoware::map_projection_loader::MapProjectionLoader',
                name='map_projection_loader', namespace='/atlas_map_demo',
                parameters=[{'map_projector_info_path': projector, 'lanelet2_map_path': osm}]),
            ComposableNode(
                package='autoware_map_loader',
                plugin='autoware::map_loader::Lanelet2MapLoaderNode',
                name='lanelet2_map_loader', namespace='/atlas_map_demo',
                parameters=[{'lanelet2_map_path': osm,
                             'allow_unsupported_version': True,
                             'center_line_resolution': 1.0, 'use_waypoints': False}],
                remappings=[('/map/vector_map', '/atlas_map_demo/vector_map')]),
            ComposableNode(
                package='autoware_map_loader',
                plugin='autoware::map_loader::PointCloudMapLoaderNode',
                name='pointcloud_map_loader', namespace='/atlas_map_demo',
                parameters=[{'pcd_paths_or_directory': [[pcd]],
                             'pcd_metadata_path': '/tmp/atlas_map_demo_no_metadata.yaml',
                             'enable_whole_load': True,
                             'enable_downsampled_whole_load': False,
                             'enable_partial_load': False, 'enable_selected_load': False}],
                remappings=[('output/pointcloud_map', 'pointcloud_map')]),
            ComposableNode(
                package='autoware_lanelet2_map_visualizer',
                plugin='autoware::lanelet2_map_visualizer::Lanelet2MapVisualizationNode',
                name='lanelet2_map_visualization', namespace='/atlas_map_demo',
                remappings=[('input/lanelet2_map', 'vector_map'),
                            ('output/lanelet2_map_marker', 'road_markers')]),
        ])
    probe = Node(package='atlas_map_integration_demo', executable='map_routing_probe',
                 name='map_routing_probe', namespace='/atlas_map_demo', output='screen',
                 parameters=[{'results_dir': LaunchConfiguration('results_dir')}])
    rviz = Node(package='rviz2', executable='rviz2', name='campus_rviz',
                namespace='/atlas_map_demo', output='screen',
                arguments=['-d', os.path.join(share, 'config', 'campus_map.rviz')],
                condition=IfCondition(LaunchConfiguration('launch_rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('osm_path', default_value=os.path.join(os.environ.get('ATLAS_MAP_DIR', os.path.expanduser('~/Downloads')), 'finalMap_repaired.osm')),
        DeclareLaunchArgument('pcd_path', default_value=os.path.join(os.environ.get('ATLAS_MAP_DIR', os.path.expanduser('~/Downloads')), 'pointcloud_map_georeferenced.pcd')),
        DeclareLaunchArgument('projector_path', default_value=os.path.join(os.environ.get('ATLAS_MAP_DIR', os.path.expanduser('~/Downloads')), 'map_projector_info.yaml')),
        DeclareLaunchArgument('results_dir', default_value='/tmp/atlas_map_demo_results'),
        DeclareLaunchArgument('launch_rviz', default_value='true'), container, probe, rviz])
