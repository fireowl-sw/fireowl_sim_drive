#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    new_scan_arg = DeclareLaunchArgument(
        'new_scan',
        default_value='true',
        description='Enable new scan'
    )

    vehicle_radius_arg = DeclareLaunchArgument(
        'vehicle_radius',
        default_value='8.0',
        description='Vehicle radius for filtering points'
    )

    cloud_resolution_arg = DeclareLaunchArgument(
        'cloud_resolution',
        default_value='5000',
        description='Cloud resolution threshold'
    )

    # Get RViz config path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('carla_lidar_mapping_ros2'),
        'rviz',
        'mapping.rviz'
    ])

    # Lidar mapping node
    lidar_mapping_node = Node(
        package='carla_lidar_mapping_ros2',
        executable='lidar_mapping_node',
        name='lidar_mapping',
        output='screen',
        parameters=[{
            'new_scan': LaunchConfiguration('new_scan'),
            'vehicle_radius': LaunchConfiguration('vehicle_radius'),
            'cloud_resolution': LaunchConfiguration('cloud_resolution'),
        }]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        new_scan_arg,
        vehicle_radius_arg,
        cloud_resolution_arg,
        lidar_mapping_node,
        rviz_node,
    ])
