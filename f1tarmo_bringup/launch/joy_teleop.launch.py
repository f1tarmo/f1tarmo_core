# Launch file for teleoperating the f1tarmo using a gamepad / console
# controller. 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        ),
        # Include f1tarmo_bringup to bring up the rest of the f1tarmo's core
        # nodes.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory('f1tarmo_bringup')) / 'launch' / 'f1tarmo.launch.py')
            )
        ),

    ])