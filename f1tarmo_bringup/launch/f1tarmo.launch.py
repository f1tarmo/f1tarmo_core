# Main launch file for bringing up all the core nodes for running an f1tarmo
# car.

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(Path(get_package_share_directory('f1tarmo_bringup')) / 'params' / 'f1tarmo_params.yaml'),
            description='Path to config file with parameters for the vesc nodes brought up in this launch file'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for all nodes in this launch file'
        ),
        # Include f1tarmo_description launch file to provide static transforms.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('f1tarmo_description')) / 'launch' / 'description.launch.py')
            )
        ),
        # Bring up vehicle controller / command interface node.
        Node(
            package='f1tarmo_controllers',
            executable='bicycle_controller_node',
            name='bicycle_controller_node',
            parameters=[LaunchConfiguration('params_file')]
        ),
        # Include f1tarmo_controllers launch file to provide bicycle controller
        # node.
        # TODO: Update to use composition!!!
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[LaunchConfiguration('params_file')]
        ),
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            parameters=[LaunchConfiguration('params_file')]
        ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])