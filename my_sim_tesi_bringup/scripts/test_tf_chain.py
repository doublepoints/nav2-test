#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TFChainTester(Node):
    def __init__(self):
        super().__init__('tf_chain_tester')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 测试定时器
        self.timer = self.create_timer(2.0, self.test_transforms)
        
        self.get_logger().info('TF Chain Tester started...')
        
    def test_transforms(self):
        try:
            # 测试关键的TF链
            transforms_to_test = [
                ('map', 'odom'),
                ('odom', 'base_link'),
                ('base_link', 'livox_frame_corrected'),
                ('map', 'livox_frame_corrected'),  # 端到端测试
            ]
            
            self.get_logger().info('=== TF Chain Test ===')
            
            for source, target in transforms_to_test:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        source, target, rclpy.time.Time())
                    self.get_logger().info(f'✓ {source} → {target}: OK')
                except Exception as e:
                    self.get_logger().error(f'✗ {source} → {target}: {str(e)}')
                    
        except Exception as e:
            self.get_logger().error(f'TF test error: {e}')

def main():
    rclpy.init()
    node = TFChainTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()