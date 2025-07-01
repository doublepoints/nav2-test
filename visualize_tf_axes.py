#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformListener, Buffer
import numpy as np

class TFAxesVisualizer(Node):
    def __init__(self):
        super().__init__('tf_axes_visualizer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/tf_axes', 10)
        self.create_timer(0.1, self.publish_axes)
        
        self.frames = ['base_link', 'livox_frame', 'livox_frame_corrected']
        self.colors = {
            'x': ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # Red
            'y': ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Green
            'z': ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Blue
        }
        
    def publish_axes(self):
        marker_array = MarkerArray()
        marker_id = 0
        
        for frame in self.frames:
            try:
                # 获取 frame 到 map 的变换
                transform = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                
                # 创建坐标轴标记
                for axis, color in [('x', self.colors['x']), 
                                   ('y', self.colors['y']), 
                                   ('z', self.colors['z'])]:
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = f'{frame}_axes'
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD
                    
                    # 设置箭头起点和终点
                    start = Point()
                    start.x = transform.transform.translation.x
                    start.y = transform.transform.translation.y
                    start.z = transform.transform.translation.z
                    
                    end = Point()
                    end.x = start.x
                    end.y = start.y
                    end.z = start.z
                    
                    # 根据轴向设置终点
                    length = 0.5  # 箭头长度
                    if axis == 'x':
                        end.x += length
                    elif axis == 'y':
                        end.y += length
                    elif axis == 'z':
                        end.z += length
                    
                    marker.points = [start, end]
                    marker.scale.x = 0.02  # 箭头粗细
                    marker.scale.y = 0.05  # 箭头头部大小
                    marker.color = color
                    
                    marker_array.markers.append(marker)
                    
                # 添加文本标签
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
                text_marker.pose.position.z = transform.transform.translation.z + 0.3
                text_marker.text = frame
                text_marker.scale.z = 0.1
                text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                marker_array.markers.append(text_marker)
                
            except Exception as e:
                pass  # 忽略变换不可用的情况
        
        self.marker_pub.publish(marker_array)
        
def main(args=None):
    rclpy.init(args=args)
    node = TFAxesVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()