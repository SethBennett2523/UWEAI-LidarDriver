    #!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('hesai_lidar_driver')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'hesai_visualization.rviz')
    
    # Test point cloud publisher (Python script)
    test_publisher = ExecuteProcess(
        cmd=['python3', '/home/seth/Documents/PersonalProjects/SethFSAI/UWEAI-LidarDriver/hesai/scripts/test_point_cloud_publisher.py'],
        output='screen'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        test_publisher,
        rviz_node
    ])
