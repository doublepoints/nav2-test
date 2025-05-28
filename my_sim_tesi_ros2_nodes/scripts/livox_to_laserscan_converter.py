#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

import numpy as np
import struct
import math

from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class LivoxToLaserScanConverter(Node):
    """
    将 Livox /livox/lidar PointCloud2 数据转换为 /g1/laserscan LaserScan 数据的节点
    """
    
    def __init__(self):
        super().__init__('livox_to_laserscan_converter')
        
        # 声明参数
        self.declare_parameters()
        
        # 加载参数
        self.load_parameters()
        
        # 创建QoS配置
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 创建订阅者和发布者
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pointcloud_callback,
            sensor_qos
        )
        
        self.laserscan_publisher = self.create_publisher(
            LaserScan,
            '/g1/laserscan',
            sensor_qos
        )
        
        # 统计信息
        self.msg_count = 0
        self.last_log_time = self.get_clock().now()
        
        self.get_logger().info("Livox to LaserScan Converter initialized")
        self.get_logger().info(f"Subscribed to: /livox/lidar")
        self.get_logger().info(f"Publishing to: /g1/laserscan")
        self.get_logger().info(f"Parameters:")
        self.get_logger().info(f"  Height range: [{self.min_height:.2f}, {self.max_height:.2f}] m")
        self.get_logger().info(f"  Angle range: [{math.degrees(self.angle_min):.1f}°, {math.degrees(self.angle_max):.1f}°]")
        self.get_logger().info(f"  Range: [{self.range_min:.2f}, {self.range_max:.2f}] m")
        self.get_logger().info(f"  Angular resolution: {math.degrees(self.angle_increment):.2f}°")
    
    def declare_parameters(self):
        """声明节点参数"""
        # 高度过滤范围 (米)
        self.declare_parameter('min_height', -1.0)
        self.declare_parameter('max_height', 2.0)
        
        # 角度范围 (弧度)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.radians(0.25))  # 0.25度
        
        # 距离范围 (米)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 100.0)
        
        # 扫描参数
        self.declare_parameter('scan_time', 0.1)  # 10Hz
        self.declare_parameter('time_increment', 0.0)
        
        # 无效值处理
        self.declare_parameter('use_inf', True)
        self.declare_parameter('inf_epsilon', 1.0)
        
        # 输出坐标系
        self.declare_parameter('output_frame_id', 'livox_frame')
        
        # 调试选项
        self.declare_parameter('debug_output', False)
        self.declare_parameter('log_interval', 10.0)  # 日志输出间隔(秒)
    
    def load_parameters(self):
        """加载参数"""
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.angle_min = self.get_parameter('angle_min').get_parameter_value().double_value
        self.angle_max = self.get_parameter('angle_max').get_parameter_value().double_value
        self.angle_increment = self.get_parameter('angle_increment').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        self.scan_time = self.get_parameter('scan_time').get_parameter_value().double_value
        self.time_increment = self.get_parameter('time_increment').get_parameter_value().double_value
        self.use_inf = self.get_parameter('use_inf').get_parameter_value().bool_value
        self.inf_epsilon = self.get_parameter('inf_epsilon').get_parameter_value().double_value
        self.output_frame_id = self.get_parameter('output_frame_id').get_parameter_value().string_value
        self.debug_output = self.get_parameter('debug_output').get_parameter_value().bool_value
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().double_value
        
        # 计算激光扫描的数组大小
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
    
    def pointcloud_callback(self, msg):
        """处理点云数据的回调函数"""
        try:
            self.msg_count += 1
            current_time = self.get_clock().now()
            
            # 定期输出日志
            if self.debug_output and (current_time - self.last_log_time).nanoseconds / 1e9 > self.log_interval:
                self.get_logger().info(f"Processed {self.msg_count} PointCloud2 messages")
                self.last_log_time = current_time
            
            # 转换点云为激光扫描
            laser_scan = self.convert_pointcloud_to_laserscan(msg)
            
            # 发布激光扫描数据
            self.laserscan_publisher.publish(laser_scan)
            
            if self.debug_output and self.msg_count % 50 == 0:  # 每50个消息输出一次详细信息
                valid_ranges = sum(1 for r in laser_scan.ranges if not math.isinf(r))
                self.get_logger().debug(f"LaserScan: {valid_ranges}/{len(laser_scan.ranges)} valid ranges")
        
        except Exception as e:
            self.get_logger().error(f"Error in pointcloud callback: {str(e)}")
    
    def convert_pointcloud_to_laserscan(self, pointcloud_msg):
        """将PointCloud2消息转换为LaserScan消息"""
        
        # 创建LaserScan消息
        laser_scan = LaserScan()
        
        # 设置头信息
        laser_scan.header = Header()
        laser_scan.header.stamp = pointcloud_msg.header.stamp
        laser_scan.header.frame_id = self.output_frame_id
        
        # 设置扫描参数
        laser_scan.angle_min = self.angle_min
        laser_scan.angle_max = self.angle_max
        laser_scan.angle_increment = self.angle_increment
        laser_scan.time_increment = self.time_increment
        laser_scan.scan_time = self.scan_time
        laser_scan.range_min = self.range_min
        laser_scan.range_max = self.range_max
        
        # 初始化距离和强度数组
        laser_scan.ranges = [float('inf')] * self.num_ranges
        laser_scan.intensities = [0.0] * self.num_ranges
        
        # 解析点云数据
        try:
            points = self.extract_points_from_pointcloud(pointcloud_msg)
            
            # 处理每个点
            for point in points:
                x, y, z, intensity = point
                
                # 高度过滤
                if z < self.min_height or z > self.max_height:
                    continue
                
                # 计算距离和角度
                range_val = math.sqrt(x * x + y * y)
                angle = math.atan2(y, x)
                
                # 距离过滤
                if range_val < self.range_min or range_val > self.range_max:
                    continue
                
                # 角度过滤
                if angle < self.angle_min or angle > self.angle_max:
                    continue
                
                # 计算在激光扫描数组中的索引
                angle_index = int((angle - self.angle_min) / self.angle_increment)
                
                # 确保索引在有效范围内
                if 0 <= angle_index < self.num_ranges:
                    # 如果该角度位置没有距离值，或者当前距离更近，则更新
                    if (math.isinf(laser_scan.ranges[angle_index]) or 
                        range_val < laser_scan.ranges[angle_index]):
                        laser_scan.ranges[angle_index] = range_val
                        laser_scan.intensities[angle_index] = intensity
        
        except Exception as e:
            self.get_logger().error(f"Error processing pointcloud: {str(e)}")
            return laser_scan
        
        # 处理无效距离值
        if not self.use_inf:
            for i in range(len(laser_scan.ranges)):
                if math.isinf(laser_scan.ranges[i]):
                    laser_scan.ranges[i] = self.range_max + self.inf_epsilon
        
        return laser_scan
    
    def extract_points_from_pointcloud(self, pointcloud_msg):
        """从PointCloud2消息中提取点数据"""
        points = []
        
        try:
            # 使用sensor_msgs_py库来解析点云数据
            for point in point_cloud2.read_points(
                pointcloud_msg, 
                field_names=('x', 'y', 'z', 'intensity'), 
                skip_nans=True
            ):
                x, y, z, intensity = point
                points.append((x, y, z, intensity if intensity is not None else 0.0))
        
        except Exception as e:
            self.get_logger().warn(f"Failed to read points with intensity, trying without: {str(e)}")
            try:
                # 如果没有强度信息，只读取xyz
                for point in point_cloud2.read_points(
                    pointcloud_msg, 
                    field_names=('x', 'y', 'z'), 
                    skip_nans=True
                ):
                    x, y, z = point
                    points.append((x, y, z, 0.0))
            except Exception as e2:
                self.get_logger().error(f"Failed to read points: {str(e2)}")
                # 尝试手动解析
                points = self.manual_parse_pointcloud(pointcloud_msg)
        
        return points
    
    def manual_parse_pointcloud(self, pointcloud_msg):
        """手动解析PointCloud2数据（备用方法）"""
        points = []
        
        try:
            # 获取字段信息
            x_offset = y_offset = z_offset = intensity_offset = None
            for field in pointcloud_msg.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
                elif field.name == 'intensity':
                    intensity_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().error("Required fields (x, y, z) not found in PointCloud2")
                return points
            
            # 解析数据
            point_step = pointcloud_msg.point_step
            data = pointcloud_msg.data
            
            for i in range(0, len(data), point_step):
                try:
                    # 提取x, y, z坐标 (假设为float32)
                    x = struct.unpack_from('f', data, i + x_offset)[0]
                    y = struct.unpack_from('f', data, i + y_offset)[0]
                    z = struct.unpack_from('f', data, i + z_offset)[0]
                    
                    # 检查是否为有效值
                    if math.isnan(x) or math.isnan(y) or math.isnan(z):
                        continue
                    
                    # 提取强度信息（如果存在）
                    intensity = 0.0
                    if intensity_offset is not None:
                        try:
                            intensity = struct.unpack_from('f', data, i + intensity_offset)[0]
                            if math.isnan(intensity):
                                intensity = 0.0
                        except:
                            intensity = 0.0
                    
                    points.append((x, y, z, intensity))
                
                except Exception as e:
                    continue  # 跳过无效点
            
            self.get_logger().debug(f"Manually parsed {len(points)} points from PointCloud2")
        
        except Exception as e:
            self.get_logger().error(f"Manual parsing failed: {str(e)}")
        
        return points


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LivoxToLaserScanConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
