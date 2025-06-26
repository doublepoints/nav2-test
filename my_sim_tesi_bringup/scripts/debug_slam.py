#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time

class SLAMDebugger(Node):
    def __init__(self):
        super().__init__('slam_debugger')
        
        # QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # 订阅所有相关topics
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, '/g1/laserscan', self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, '/g1/odometry', self.odom_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        
        # 计数器
        self.scan_count = 0
        self.map_count = 0
        self.odom_count = 0
        self.pose_count = 0
        
        # 定时报告
        self.timer = self.create_timer(3.0, self.report_status)
        self.start_time = time.time()
        
        self.get_logger().info('SLAM Debugger started...')
        
    def map_callback(self, msg):
        self.map_count += 1
        self.get_logger().info(f'Map #{self.map_count}: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution}')
        
    def scan_callback(self, msg):
        self.scan_count += 1
        if self.scan_count % 10 == 0:  # 每10个scan报告一次
            self.get_logger().info(f'Scan #{self.scan_count}: frame={msg.header.frame_id}, ranges={len(msg.ranges)}')
        
    def odom_callback(self, msg):
        self.odom_count += 1
        if self.odom_count % 10 == 0:
            pos = msg.pose.pose.position
            self.get_logger().info(f'Odom #{self.odom_count}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')
        
    def pose_callback(self, msg):
        self.pose_count += 1
        pos = msg.pose.pose.position
        self.get_logger().info(f'SLAM Pose #{self.pose_count}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})')
        
    def report_status(self):
        elapsed = time.time() - self.start_time
        self.get_logger().info(f'=== Status after {elapsed:.1f}s ===')
        self.get_logger().info(f'  Maps: {self.map_count}')
        self.get_logger().info(f'  Scans: {self.scan_count}')  
        self.get_logger().info(f'  Odoms: {self.odom_count}')
        self.get_logger().info(f'  SLAM Poses: {self.pose_count}')
        
def main():
    rclpy.init()
    node = SLAMDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()