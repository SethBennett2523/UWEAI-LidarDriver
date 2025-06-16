#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hesai_lidar_driver',
            executable='hesai_lidar_node',
            name='hesai_lidar_node',
            output='screen',
            parameters=[{
                'device_ip': '192.168.3.210',
                'data_port': 2368,
                'position_port': 8308,
                'frame_id': 'hesai_lidar',
                'scan_time': 0.1,
                'calibration_file': '',
            }]
        )
    ])
