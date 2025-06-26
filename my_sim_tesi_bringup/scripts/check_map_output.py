#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapChecker(Node):
    def __init__(self):
        super().__init__('map_checker')
        self.map_received = False
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.get_logger().info('Waiting for /map topic...')
        
        # 创建定时器，每5秒报告状态
        self.timer = self.create_timer(5.0, self.status_callback)
        
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info('✓ Map received!')
        
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        
        # 计算地图中的占用、空闲和未知区域
        occupied = sum(1 for cell in msg.data if cell > 50)
        free = sum(1 for cell in msg.data if 0 <= cell <= 50)
        unknown = sum(1 for cell in msg.data if cell < 0)
        
        self.get_logger().info(
            f'Map update: {width}x{height} @ {resolution}m/px | '
            f'Occupied: {occupied}, Free: {free}, Unknown: {unknown}'
        )
        
    def status_callback(self):
        if not self.map_received:
            self.get_logger().warn('No map received yet...')
        
def main(args=None):
    rclpy.init(args=args)
    node = MapChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()