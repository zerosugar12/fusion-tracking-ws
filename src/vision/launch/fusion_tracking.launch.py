import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('vision')
    default_params = os.path.join(pkg_dir, 'config', 'params.yaml')
    default_rviz = os.path.join(pkg_dir, 'rviz', 'fusion_tracking.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to ROS 2 parameters YAML file'
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='Whether to automatically launch RViz2'
        ),
        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='Whether to log benchmark CSV telemetry for thesis'
        ),

        Node(
            package='vision',
            executable='fusion_tracker_node',
            name='fusion_tracker_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'enable_csv_logging': LaunchConfiguration('enable_logging')}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_fusion',
            output='screen',
            arguments=['-d', default_rviz],
            condition=IfCondition(LaunchConfiguration('enable_rviz'))
        ),
    ])
