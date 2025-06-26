#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class TimestampChecker(Node):
    def __init__(self):
        super().__init__('timestamp_checker')
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribe to topics
        self.scan_sub = self.create_subscription(
            LaserScan, '/g1/laserscan', self.scan_callback, sensor_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/g1/odometry', self.odom_callback, 10)
        
        self.last_scan_time = None
        self.last_odom_time = None
        self.scan_count = 0
        self.odom_count = 0
        
        # Report timer
        self.timer = self.create_timer(2.0, self.report_timestamps)
        
        self.get_logger().info('Timestamp Checker started...')
        
    def scan_callback(self, msg):
        self.scan_count += 1
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.scan_count == 1:
            self.get_logger().info(f'First scan timestamp: {current_time:.3f}')
        
        if self.last_scan_time:
            dt = current_time - self.last_scan_time
            if abs(dt) > 1.0:  # 时间跳变超过1秒
                self.get_logger().warn(f'Scan time jump: {dt:.3f}s')
        
        self.last_scan_time = current_time
        
    def odom_callback(self, msg):
        self.odom_count += 1
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.odom_count == 1:
            self.get_logger().info(f'First odom timestamp: {current_time:.3f}')
            
        if self.last_odom_time:
            dt = current_time - self.last_odom_time
            if abs(dt) > 1.0:  # 时间跳变超过1秒
                self.get_logger().warn(f'Odom time jump: {dt:.3f}s')
                
        self.last_odom_time = current_time
        
    def report_timestamps(self):
        if self.last_scan_time and self.last_odom_time:
            time_diff = abs(self.last_scan_time - self.last_odom_time)
            system_time = time.time()
            
            self.get_logger().info(f'=== Timestamp Report ===')
            self.get_logger().info(f'System time: {system_time:.3f}')
            self.get_logger().info(f'Last scan time: {self.last_scan_time:.3f}')
            self.get_logger().info(f'Last odom time: {self.last_odom_time:.3f}')
            self.get_logger().info(f'Time difference: {time_diff:.3f}s')
            self.get_logger().info(f'Scan count: {self.scan_count}, Odom count: {self.odom_count}')
            
            if time_diff > 0.5:
                self.get_logger().warn(f'WARNING: Large time difference between scan and odom!')

def main():
    rclpy.init()
    node = TimestampChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()