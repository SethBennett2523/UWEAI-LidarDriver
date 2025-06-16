#!/usr/bin/env python3

"""
Hesai LiDAR Mapping Demonstration Launch File

WARNING: THIS IS FOR DEMONSTRATION PURPOSES ONLY

This launch configuration uses optimised point persistence (0.15 seconds) to 
visualise complete 360° LiDAR coverage. Points persist just long enough to 
show the full scan pattern before being replaced by new data.

This creates a real-time mapping view that shows the complete LiDAR coverage
while still being suitable for demonstration purposes.

Author: LiDAR Driver Development Team
Date: 16 June 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description for Hesai LiDAR mapping demonstration.
    
    This configuration shows accumulated point clouds over time to demonstrate
    the mapping capability. It is NOT suitable for real-time applications.
    """
    
    # Get package directory
    package_dir = get_package_share_directory('hesai_lidar_driver')
    
    # RViz configuration file
    rviz_config_file = os.path.join(package_dir, 'rviz', 'hesai_mapping_demo.rviz')
    
    # Declare launch arguments
    device_ip_arg = DeclareLaunchArgument(
        'device_ip',
        default_value='192.168.3.210',
        description='IP address of the Hesai LiDAR device'
    )
    
    data_port_arg = DeclareLaunchArgument(
        'data_port', 
        default_value='2368',
        description='UDP port for LiDAR data packets'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='hesai_lidar', 
        description='TF frame ID for the LiDAR sensor'
    )
    
    # Warning message about demonstration mode
    demo_warning = LogInfo(
        msg='='*80 + '\n' +
            'HESAI LIDAR MAPPING DEMONSTRATION MODE\n' +
            'WARNING: This configuration uses optimised point persistence\n' +
            'to show complete 360° LiDAR coverage for demonstration.\n' +
            'For production applications, use hesai_visualization.launch.py\n' +
            '='*80
    )
    
    # Hesai LiDAR demonstration driver node (with warning messages)
    lidar_driver_node = Node(
        package='hesai_lidar_driver',
        executable='hesai_demo_node',  # Use demo version with warnings
        name='hesai_demo_node',
        output='screen',
        parameters=[{
            'device_ip': LaunchConfiguration('device_ip'),
            'data_port': LaunchConfiguration('data_port'),
            'position_port': 8308,
            'frame_id': LaunchConfiguration('frame_id'),
            'scan_time': 0.1,
            'calibration_file': ''
        }],
        remappings=[
            ('hesai_lidar/raw_cloud', '/hesai_lidar/raw_cloud')
        ]
    )
    
    # RViz2 visualisation node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        device_ip_arg,
        data_port_arg, 
        frame_id_arg,
        demo_warning,
        lidar_driver_node,
        rviz_node
    ])
