#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class RVizMapTester(Node):
    def __init__(self):
        super().__init__('rviz_map_tester')
        
        # 使用正确的QoS配置订阅地图
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            map_qos
        )
        
        self.map_count = 0
        self.get_logger().info('🗺️ Map tester started - listening for map messages...')
        
    def map_callback(self, msg):
        self.map_count += 1
        info = msg.info
        self.get_logger().info(
            f'📊 Map #{self.map_count}: {info.width}x{info.height}, '
            f'resolution={info.resolution:.3f}, frame={msg.header.frame_id}'
        )
        
        # 统计地图占用情况
        occupied = sum(1 for cell in msg.data if cell > 50)
        free = sum(1 for cell in msg.data if cell >= 0 and cell <= 50)
        unknown = sum(1 for cell in msg.data if cell < 0)
        
        self.get_logger().info(
            f'🏠 Cells: Occupied={occupied}, Free={free}, Unknown={unknown}'
        )

def main():
    rclpy.init()
    node = RVizMapTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()