#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import math

class TestPointCloudPublisher(Node):
    """
    发布测试用的 PointCloud2 数据到 /livox/lidar
    """
    
    def __init__(self):
        super().__init__('test_pointcloud_publisher')
        
        self.publisher = self.create_publisher(
            PointCloud2,
            '/livox/lidar',
            10
        )
        
        # 每秒发布一次
        self.timer = self.create_timer(1.0, self.publish_pointcloud)
        
        self.seq = 0
        self.get_logger().info("Test PointCloud2 Publisher started")
        self.get_logger().info("Publishing to: /livox/lidar")
    
    def publish_pointcloud(self):
        """发布测试点云数据"""
        msg = PointCloud2()
        
        # 设置头信息
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "livox_frame"
        
        # 生成测试点云数据
        points = self.generate_test_points()
        
        # 设置字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # 4 fields * 4 bytes each
        msg.row_step = msg.point_step * len(points)
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        
        # 打包数据
        data = []
        for point in points:
            data.extend(struct.pack('ffff', *point))
        
        msg.data = data
        
        self.publisher.publish(msg)
        self.seq += 1
        
        self.get_logger().info(f"Published PointCloud2 #{self.seq} with {len(points)} points")
    
    def generate_test_points(self):
        """生成测试点云数据 - 创建一个简单的扇形扫描模式"""
        points = []
        
        # 创建扇形点云
        for angle in np.linspace(-math.pi, math.pi, 360):  # 360个点，覆盖360度
            for distance in [1.0, 2.0, 5.0, 10.0]:  # 不同距离的点
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                z = 0.1  # 稍微高于地面
                intensity = 100.0
                
                points.append([x, y, z, intensity])
        
        # 添加一些高度变化的点
        for angle in np.linspace(-math.pi/4, math.pi/4, 90):  # 前方90度
            for height in [-0.5, 0.5, 1.0]:  # 不同高度
                distance = 3.0
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                z = height
                intensity = 150.0
                
                points.append([x, y, z, intensity])
        
        return points


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestPointCloudPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
