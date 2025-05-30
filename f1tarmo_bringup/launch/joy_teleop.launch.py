from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(Path(get_package_share_directory('f1tarmo_bringup')) / 'params' / 'f1tarmo_params.yaml'),
            description='Path to config file with parameters for the vesc nodes brought up in this launch file'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joystick_interface_node',
            parameters=[LaunchConfiguration('params_file')]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='joystick_to_twist_node',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])