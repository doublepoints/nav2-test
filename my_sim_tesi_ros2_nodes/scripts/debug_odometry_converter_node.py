#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

print("🚀 调试版本启动...")

try:
    from unitree_go.msg import SportModeState
    print("✓ unitree_go.msg.SportModeState 导入成功")
    UNITREE_MSG_AVAILABLE = True
except ImportError as e:
    print(f"✗ unitree_go.msg.SportModeState 导入失败: {e}")
    UNITREE_MSG_AVAILABLE = False

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')
        print("✓ 节点创建成功")
        
        self.count = 0
        self.create_timer(2.0, self.timer_callback)
        print("✓ 定时器创建成功")
        
        if UNITREE_MSG_AVAILABLE:
            self.subscription = self.create_subscription(
                SportModeState,
                '/lf/odommodestate',
                self.callback,
                10
            )
            print("✓ 订阅者创建成功: /lf/odommodestate")
        else:
            print("⚠ 无法创建订阅者，unitree_go包不可用")
    
    def callback(self, msg):
        self.count += 1
        print(f"📨 收到消息 #{self.count}")
        if self.count <= 3:
            print(f"   位置: {msg.position[:3]}")
    
    def timer_callback(self):
        print(f"⏰ 定时器触发，已收到 {self.count} 个消息")
        if self.count == 0:
            print("   💡 提示：如果没有消息，请检查rosbag是否在播放")

def main():
    print("初始化ROS2...")
    rclpy.init()
    node = DebugNode()
    print("开始运行，按Ctrl+C停止...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n停止运行")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
