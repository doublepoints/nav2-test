#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time

class QuickTest(Node):
    def __init__(self):
        super().__init__('quick_test')
        
        self.map_received = False
        self.scan_received = False
        self.odom_received = False
        
        # 创建QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # 订阅关键topics
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, '/g1/laserscan', self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/g1/odometry', self.odom_callback, 10)
        
        # 定时报告
        self.timer = self.create_timer(5.0, self.report_status)
        self.start_time = time.time()
        
        self.get_logger().info('Quick test started - monitoring key topics...')
        
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info('✓ Map topic working!')
        
    def scan_callback(self, msg):
        if not self.scan_received:
            self.scan_received = True
            self.get_logger().info(f'✓ Scan topic working! Frame: {msg.header.frame_id}')
        
    def odom_callback(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(f'✓ Odometry topic working! Frame: {msg.header.frame_id}')
        
    def report_status(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f'Status after {elapsed:.1f}s:')
        self.get_logger().info(f'  Map: {"✓" if self.map_received else "✗"}')
        self.get_logger().info(f'  Scan: {"✓" if self.scan_received else "✗"}')  
        self.get_logger().info(f'  Odom: {"✓" if self.odom_received else "✗"}')
        
        if all([self.map_received, self.scan_received, self.odom_received]):
            self.get_logger().info('🎉 All systems working!')
            
def main():
    rclpy.init()
    node = QuickTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()