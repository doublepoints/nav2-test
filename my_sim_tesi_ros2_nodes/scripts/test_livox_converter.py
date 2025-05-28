#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LivexConverterTester(Node):
    """
    测试Livox转换器输出的节点
    """
    
    def __init__(self):
        super().__init__('livox_converter_tester')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/g1/laserscan',
            self.laserscan_callback,
            10
        )
        
        self.msg_count = 0
        self.get_logger().info("Livox Converter Tester started. Listening on /g1/laserscan...")
    
    def laserscan_callback(self, msg):
        """处理接收到的LaserScan消息"""
        self.msg_count += 1
        
        # 计算统计信息
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        total_ranges = len(msg.ranges)
        valid_count = len(valid_ranges)
        
        if valid_count > 0:
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            avg_range = sum(valid_ranges) / valid_count
        else:
            min_range = max_range = avg_range = 0.0
        
        # 每10条消息输出一次统计信息
        if self.msg_count % 10 == 0:
            self.get_logger().info(
                f"LaserScan #{self.msg_count}:\n"
                f"  Frame: {msg.header.frame_id}\n"
                f"  Valid ranges: {valid_count}/{total_ranges}\n"
                f"  Range stats: min={min_range:.2f}m, max={max_range:.2f}m, avg={avg_range:.2f}m\n"
                f"  Angle range: [{math.degrees(msg.angle_min):.1f}°, {math.degrees(msg.angle_max):.1f}°]\n"
                f"  Angular resolution: {math.degrees(msg.angle_increment):.2f}°"
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LivexConverterTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
