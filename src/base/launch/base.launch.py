import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('base')
    default_params = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to ROS 2 parameters YAML file for base node'
        ),

        Node(
            package='base',
            executable='nucleo',
            name='nucleo',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
