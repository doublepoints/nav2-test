#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import time

class TFDiagnosticNode(Node):
    def __init__(self):
        super().__init__('tf_diagnostic_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_timer(1.0, self.check_transforms)
        
    def check_transforms(self):
        try:
            # 检查 odom -> base_link
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            self.get_logger().info(
                f'odom -> base_link: x={x:.3f}, y={y:.3f}, z={z:.3f}'
            )
            
            # 检查是否存在 map -> odom
            try:
                map_to_odom = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
                mx = map_to_odom.transform.translation.x
                my = map_to_odom.transform.translation.y
                mz = map_to_odom.transform.translation.z
                self.get_logger().info(
                    f'map -> odom: x={mx:.3f}, y={my:.3f}, z={mz:.3f}'
                )
            except:
                pass
                
        except Exception as e:
            self.get_logger().warn(f'Transform not available: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TFDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()