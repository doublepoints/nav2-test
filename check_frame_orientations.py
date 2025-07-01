#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import tf_transformations
import math

class FrameOrientationChecker(Node):
    def __init__(self):
        super().__init__('frame_orientation_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_timer(2.0, self.check_orientations)
        
    def check_orientations(self):
        frames = [
            ('base_link', 'livox_frame'),
            ('livox_frame', 'livox_frame_corrected'),
            ('base_link', 'livox_frame_corrected')
        ]
        
        for parent, child in frames:
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                
                # 提取四元数
                q = transform.transform.rotation
                quaternion = [q.x, q.y, q.z, q.w]
                
                # 转换为欧拉角
                euler = tf_transformations.euler_from_quaternion(quaternion)
                euler_deg = [math.degrees(angle) for angle in euler]
                
                self.get_logger().info(
                    f'{parent} -> {child}:\n'
                    f'  Quaternion: [{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]\n'
                    f'  Euler (deg): Roll={euler_deg[0]:.1f}°, Pitch={euler_deg[1]:.1f}°, Yaw={euler_deg[2]:.1f}°\n'
                )
                
            except Exception as e:
                self.get_logger().warn(f'Cannot get {parent} -> {child}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FrameOrientationChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()