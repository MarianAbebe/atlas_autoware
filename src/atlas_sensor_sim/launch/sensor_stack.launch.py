from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    default = get_package_share_directory('atlas_sensor_sim') + '/config/sensor_sim.yaml'
    args=[DeclareLaunchArgument('params_file', default_value=default), DeclareLaunchArgument('scenario', default_value='deterministic')]
    # scenario is intentionally named for repeatable CI; geometry is fixed in v0.1.
    common={'parameters':[LaunchConfiguration('params_file')]}
    return LaunchDescription(args+[Node(package='atlas_sensor_sim',executable='sensor_simulator',name='atlas_sensor_simulator',output='screen',**common),Node(package='atlas_sensor_sim',executable='sensor_health_monitor',name='atlas_sensor_health_monitor',output='screen',**common)])
