#!/usr/bin/env python3
"""
测试Unitree SportModeState到标准Odometry的转换
使用方法: 
1. 播放rosbag: ros2 bag play /path/to/rosbag --clock
2. 运行此脚本: python3 test_odometry_conversion.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys

try:
    from unitree_go.msg import SportModeState
    UNITREE_MSG_AVAILABLE = True
except ImportError:
    UNITREE_MSG_AVAILABLE = False
    print("Warning: unitree_go.msg.SportModeState not available")

class ConversionTester(Node):
    def __init__(self):
        super().__init__('conversion_tester')
        
        # 安全地声明use_sim_time参数
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            # 参数已经存在，这是正常的
            pass
        
        self.sport_mode_count = 0
        self.odom_count = 0
        self.last_sport_mode_msg = None
        self.last_odom_msg = None
        
        # 订阅原始消息
        if UNITREE_MSG_AVAILABLE:
            self.sport_mode_sub = self.create_subscription(
                SportModeState,
                '/lf/odommodestate',
                self.sport_mode_callback,
                10
            )
        
        # 订阅转换后的消息
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 创建定时器进行状态报告
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info("=== Odometry转换测试器启动 ===")
        if not UNITREE_MSG_AVAILABLE:
            self.get_logger().warn("unitree_go包不可用，只能检查转换后的消息")
    
    def sport_mode_callback(self, msg):
        self.sport_mode_count += 1
        self.last_sport_mode_msg = msg
        
        if self.sport_mode_count == 1:
            self.get_logger().info("✓ 收到第一个SportModeState消息")
            self.print_sport_mode_details(msg)