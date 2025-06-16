#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

class TestPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('test_point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'hesai_lidar/raw_cloud', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.get_logger().info('Test Point Cloud Publisher started')

    def create_test_pointcloud(self):
        # Create a simple test point cloud (cylinder of points)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'hesai_lidar'
        
        # Generate test points in a cylinder pattern
        points = []
        for angle in np.linspace(0, 2*np.pi, 36):  # 36 points in circle
            for z in np.linspace(-2, 2, 10):  # 10 height levels
                x = 5.0 * np.cos(angle)  # 5m radius
                y = 5.0 * np.sin(angle)
                intensity = 100.0
                points.append([x, y, z, intensity])
        
        points = np.array(points, dtype=np.float32)
        
        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        # Pack the point data
        cloud_data = []
        for point in points:
            cloud_data.extend(struct.pack('ffff', point[0], point[1], point[2], point[3]))
        
        cloud_msg.data = cloud_data
        return cloud_msg

    def timer_callback(self):
        msg = self.create_test_pointcloud()
        self.publisher_.publish(msg)
        # Log occasionally to show it's working
        if hasattr(self, 'count'):
            self.count += 1
        else:
            self.count = 1
        
        if self.count % 50 == 0:  # Every 5 seconds
            self.get_logger().info(f'Published {self.count} test point clouds')

def main(args=None):
    rclpy.init(args=args)
    node = TestPointCloudPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
