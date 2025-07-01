#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformListener, Buffer
import tf_transformations
import numpy as np

class CoordinateFrameVisualizer(Node):
    def __init__(self):
        super().__init__('coordinate_frame_visualizer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/coordinate_frames_debug', 10)
        self.create_timer(1.0, self.publish_coordinate_frames)
        
        self.frames = ['base_link', 'livox_frame', 'livox_frame_corrected']
        self.frame_colors = {
            'base_link': ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),        # 紫色
            'livox_frame': ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),      # 青色  
            'livox_frame_corrected': ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # 黄色
        }
        
    def publish_coordinate_frames(self):
        marker_array = MarkerArray()
        marker_id = 0
        
        try:
            # 获取各坐标系的变换关系并分析
            self.get_logger().info("=== 坐标系分析 ===")
            
            for frame in self.frames:
                try:
                    # 获取从 map 到各个 frame 的变换
                    transform = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                    
                    # 提取旋转四元数并转换为欧拉角
                    q = transform.transform.rotation
                    quaternion = [q.x, q.y, q.z, q.w]
                    euler = tf_transformations.euler_from_quaternion(quaternion)
                    euler_deg = [np.degrees(angle) for angle in euler]
                    
                    self.get_logger().info(f"{frame}:")
                    self.get_logger().info(f"  位置: ({transform.transform.translation.x:.3f}, {transform.transform.translation.y:.3f}, {transform.transform.translation.z:.3f})")
                    self.get_logger().info(f"  欧拉角 (度): Roll={euler_deg[0]:.1f}°, Pitch={euler_deg[1]:.1f}°, Yaw={euler_deg[2]:.1f}°")
                    
                    # 创建坐标轴标记
                    axes_length = 0.3
                    axis_names = ['X', 'Y', 'Z']
                    axis_colors = [
                        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # X轴 红色
                        ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Y轴 绿色  
                        ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Z轴 蓝色
                    ]
                    
                    # 计算旋转矩阵来确定轴的方向
                    rotation_matrix = tf_transformations.quaternion_matrix(quaternion)
                    
                    for i, (axis_name, axis_color) in enumerate(zip(axis_names, axis_colors)):
                        marker = Marker()
                        marker.header.frame_id = 'map'
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = f'{frame}_axis_{axis_name}'
                        marker.id = marker_id
                        marker_id += 1
                        marker.type = Marker.ARROW
                        marker.action = Marker.ADD
                        
                        # 起点
                        start = Point()
                        start.x = transform.transform.translation.x
                        start.y = transform.transform.translation.y  
                        start.z = transform.transform.translation.z
                        
                        # 终点：沿着对应轴的方向
                        axis_direction = rotation_matrix[:3, i]  # 第i列是第i个轴的方向
                        end = Point()
                        end.x = start.x + axis_direction[0] * axes_length
                        end.y = start.y + axis_direction[1] * axes_length
                        end.z = start.z + axis_direction[2] * axes_length
                        
                        marker.points = [start, end]
                        marker.scale.x = 0.02  # 箭头粗细
                        marker.scale.y = 0.04  # 箭头头部大小
                        marker.color = axis_color
                        
                        marker_array.markers.append(marker)
                    
                    # 添加坐标系标签
                    text_marker = Marker()
                    text_marker.header.frame_id = 'map'
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = f'{frame}_label'
                    text_marker.id = marker_id
                    marker_id += 1
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    text_marker.pose.position.x = transform.transform.translation.x
                    text_marker.pose.position.y = transform.transform.translation.y
                    text_marker.pose.position.z = transform.transform.translation.z + 0.4
                    text_marker.text = frame
                    text_marker.scale.z = 0.1
                    text_marker.color = self.frame_colors[frame]
                    marker_array.markers.append(text_marker)
                    
                except Exception as e:
                    self.get_logger().warn(f'无法获取 {frame} 的变换: {e}')
            
            # 分析两两之间的关系
            self.get_logger().info("=== 坐标系间的相对关系 ===")
            frame_pairs = [
                ('base_link', 'livox_frame'),
                ('livox_frame', 'livox_frame_corrected'), 
                ('base_link', 'livox_frame_corrected')
            ]
            
            for parent, child in frame_pairs:
                try:
                    rel_transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                    q = rel_transform.transform.rotation
                    quaternion = [q.x, q.y, q.z, q.w]
                    euler = tf_transformations.euler_from_quaternion(quaternion)
                    euler_deg = [np.degrees(angle) for angle in euler]
                    
                    self.get_logger().info(f"{parent} -> {child}:")
                    self.get_logger().info(f"  旋转 (度): Roll={euler_deg[0]:.1f}°, Pitch={euler_deg[1]:.1f}°, Yaw={euler_deg[2]:.1f}°")
                    
                except Exception as e:
                    self.get_logger().warn(f'无法获取 {parent} -> {child} 的变换: {e}')
                    
        except Exception as e:
            self.get_logger().error(f'处理坐标系时出错: {e}')
        
        # 发布标记
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateFrameVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()