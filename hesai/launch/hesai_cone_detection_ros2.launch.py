#!/usr/bin/env python3

"""
ROS2 launch file for Hesai LiDAR cone detection system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('hesai_lidar_driver')
    
    # Launch arguments
    device_ip_arg = DeclareLaunchArgument(
        'device_ip',
        default_value='192.168.3.210',
        description='IP address of the Hesai LiDAR sensor'
    )
    
    data_port_arg = DeclareLaunchArgument(
        'data_port',
        default_value='2368',
        description='UDP port for data packets'
    )
    
    position_port_arg = DeclareLaunchArgument(
        'position_port',
        default_value='8308',
        description='UDP port for position packets'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='hesai_lidar',
        description='Frame ID for the LiDAR sensor'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualisation'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('hesai_lidar_driver'),
            'config',
            'pandar40p_calibration.csv'
        ]),
        description='Path to calibration file'
    )
    
    # Hesai LiDAR driver node
    hesai_driver_node = Node(
        package='hesai_lidar_driver',
        executable='hesai_lidar_node',
        name='hesai_lidar_node',
        output='screen',
        parameters=[{
            'device_ip': LaunchConfiguration('device_ip'),
            'data_port': LaunchConfiguration('data_port'),
            'position_port': LaunchConfiguration('position_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }],
        remappings=[
            ('hesai_lidar/raw_cloud', 'hesai_lidar/raw_cloud'),
        ]
    )
    
    # Static transform from base_link to hesai_lidar
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hesai_lidar_transform',
        arguments=[
            '0', '0', '1.8',  # x, y, z (1.8m above base_link)
            '0', '0', '0', '1',  # qx, qy, qz, qw (no rotation)
            'base_link',
            'hesai_lidar'
        ]
    )
    
    # RViz configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('hesai_lidar_driver'),
        'rviz',
        'hesai_cone_detection.rviz'
    ])
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        device_ip_arg,
        data_port_arg,
        position_port_arg,
        frame_id_arg,
        launch_rviz_arg,
        calibration_file_arg,
        
        # Nodes
        hesai_driver_node,
        static_transform_node,
        rviz_node,
    ])
