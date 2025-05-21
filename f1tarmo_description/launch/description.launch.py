# Largely based on
# https://docs.nav2.org/setup_guides/urdf/setup_urdf.html#build-and-launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='f1tarmo_description').find('f1tarmo_description')
    default_model_path = os.path.join(pkg_share, 'model', 'f1tarmo.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(name='gui', default_value='False', description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        # Define what nodes to bring up
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    ])