#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovariance, TwistWithCovariance
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

# 导入Unitree Go2的消息类型
try:
    from unitree_go.msg import SportModeState
    UNITREE_MSG_AVAILABLE = True
except ImportError:
    UNITREE_MSG_AVAILABLE = False
    print("Warning: unitree_go.msg.SportModeState not available, using fallback")

class OdometryConverterTimeSyncNode(Node):
    def __init__(self):
        super().__init__('odometry_converter_time_sync')
        
        # 声明参数
        self.declare_parameter('input_topic', '/lf/odommodestate')
        self.declare_parameter('output_topic', '/g1/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_message_time', True)  # 新参数：是否使用消息时间
        
        # 获取参数
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.use_message_time = self.get_parameter('use_message_time').value
        
        self.get_logger().info(f'Converting from {self.input_topic} to {self.output_topic}')
        self.get_logger().info(f'Frames: {self.odom_frame} -> {self.base_frame}')
        self.get_logger().info(f'Use message time: {self.use_message_time}')
        
        # 创建发布者
        self.odom_publisher = self.create_publisher(Odometry, self.output_topic, 10)
        
        # 创建TF广播器
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # 时间同步变量
        self.first_message_time = None
        self.node_start_time = self.get_clock().now()
        
        # 创建订阅者
        if UNITREE_MSG_AVAILABLE:
            self.subscription = self.create_subscription(
                SportModeState,
                self.input_topic,
                self.sport_mode_state_callback,
                10
            )
            self.get_logger().info('Successfully subscribed to SportModeState message with time sync')
        else:
            self.get_logger().error('unitree_go.msg.SportModeState not available!')
    
    def get_synchronized_time(self, original_time_ns=None):
        """
        获取同步后的时间戳
        如果use_message_time=True且有时间戳，使用rosbag时间
        否则使用相对于第一条消息的时间偏移
        """
        current_node_time = self.get_clock().now()
        
        if self.use_message_time and original_time_ns:
            # 如果有原始时间戳，直接使用
            return rclpy.time.Time(nanoseconds=original_time_ns)
        else:
            # 使用相对时间偏移
            if self.first_message_time is None:
                self.first_message_time = current_node_time
            
            return self.first_message_time
    
    def sport_mode_state_callback(self, msg):
        """
        处理SportModeState消息的回调函数，添加时间同步
        """
        try:
            odom_msg = Odometry()
            
            # 时间同步处理
            # SportModeState消息本身可能没有时间戳，我们需要创建一个
            # 基于rosbag播放时间的时间戳
            synchronized_time = self.get_synchronized_time()
            odom_msg.header.stamp = synchronized_time.to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            # 设置位置
            odom_msg.pose.pose.position.x = float(msg.position[0])
            odom_msg.pose.pose.position.y = float(msg.position[1])
            odom_msg.pose.pose.position.z = float(msg.position[2])
            
            # 设置方向
            odom_msg.pose.pose.orientation.w = float(msg.imu_state.quaternion[0])
            odom_msg.pose.pose.orientation.x = float(msg.imu_state.quaternion[1])
            odom_msg.pose.pose.orientation.y = float(msg.imu_state.quaternion[2])
            odom_msg.pose.pose.orientation.z = float(msg.imu_state.quaternion[3])
            
            # 设置线速度
            odom_msg.twist.twist.linear.x = float(msg.velocity[0])
            odom_msg.twist.twist.linear.y = float(msg.velocity[1])
            odom_msg.twist.twist.linear.z = float(msg.velocity[2])
            
            # 设置角速度
            odom_msg.twist.twist.angular.x = float(msg.imu_state.gyroscope[0])
            odom_msg.twist.twist.angular.y = float(msg.imu_state.gyroscope[1])
            odom_msg.twist.twist.angular.z = float(msg.imu_state.gyroscope[2])
            
            # 设置协方差矩阵（估计值）
            pose_covariance = [0.1] * 36  # 6x6矩阵
            twist_covariance = [0.1] * 36  # 6x6矩阵
            odom_msg.pose.covariance = pose_covariance
            odom_msg.twist.covariance = twist_covariance
            
            # 发布里程计消息
            self.odom_publisher.publish(odom_msg)
            
            # 发布TF变换
            if self.publish_tf:
                self.publish_transform(odom_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in sport_mode_state_callback: {str(e)}')
    
    def publish_transform(self, odom_msg):
        """发布TF变换"""
        try:
            t = TransformStamped()
            t.header = odom_msg.header
            t.child_frame_id = odom_msg.child_frame_id
            
            t.transform.translation.x = odom_msg.pose.pose.position.x
            t.transform.translation.y = odom_msg.pose.pose.position.y
            t.transform.translation.z = odom_msg.pose.pose.position.z
            
            t.transform.rotation = odom_msg.pose.pose.orientation
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = OdometryConverterTimeSyncNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()