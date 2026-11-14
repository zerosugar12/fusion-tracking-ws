import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch LiDAR 360 (RPLiDAR) and static TF."""
    pkg_dir = get_package_share_directory('bringup')
    default_lidar_config = os.path.join(pkg_dir, 'config', 'lidar.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_lidar_config,
            description='Path to LiDAR configuration YAML file'
        ),
        DeclareLaunchArgument(
            'lidar_x',
            default_value='0.0',
            description='LiDAR X offset from base_link (m)'
        ),
        DeclareLaunchArgument(
            'lidar_y',
            default_value='0.0',
            description='LiDAR Y offset from base_link (m)'
        ),
        DeclareLaunchArgument(
            'lidar_z',
            default_value='0.3',
            description='LiDAR Z offset from base_link (m)'
        ),
        DeclareLaunchArgument(
            'lidar_yaw',
            default_value='3.14159',
            description='LiDAR yaw offset from base_link (rad)'
        ),
        DeclareLaunchArgument(
            'use_scan_filter',
            default_value='false',
            description='Enable laser scan angular filter'
        ),

        # RPLiDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            condition=UnlessCondition(LaunchConfiguration('use_scan_filter')),
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[('scan', 'scan_raw')],
            condition=IfCondition(LaunchConfiguration('use_scan_filter')),
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter',
            output='screen',
            remappings=[
                ('scan', 'scan_raw'),
                ('scan_filtered', 'scan'),
            ],
            parameters=[LaunchConfiguration('params_file')],
            condition=IfCondition(LaunchConfiguration('use_scan_filter')),
        ),

        # Static TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='footprint_to_base',
            output='screen',
            arguments=[
                '--frame-id', 'base_footprint',
                '--child-frame-id', 'base_link',
            ],
        ),

        # Static TF: base_link -> laser_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            output='screen',
            arguments=[
                '--x', LaunchConfiguration('lidar_x'),
                '--y', LaunchConfiguration('lidar_y'),
                '--z', LaunchConfiguration('lidar_z'),
                '--yaw', LaunchConfiguration('lidar_yaw'),
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser_frame',
            ],
        ),
    ])
