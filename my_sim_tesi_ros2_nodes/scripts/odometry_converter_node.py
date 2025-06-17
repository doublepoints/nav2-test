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

class OdometryConverterNode(Node):
    def __init__(self):
        super().__init__('odometry_converter_node')
        
        # 声明参数 - 安全地处理已存在的参数
        self.declare_parameter('input_topic', '/lf/odommodestate')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        # 安全地声明use_sim_time参数
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            # 参数已经存在，这是正常的
            pass
        
        # 获取参数
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        self.get_logger().info(f'Converting from {self.input_topic} to {self.output_topic}')
        self.get_logger().info(f'Frames: {self.odom_frame} -> {self.base_frame}')
        
        # 创建发布者
        self.odom_publisher = self.create_publisher(Odometry, self.output_topic, 10)
        
        # 创建TF广播器
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建订阅者
        if UNITREE_MSG_AVAILABLE:
            self.subscription = self.create_subscription(
                SportModeState,
                self.input_topic,
                self.sport_mode_state_callback,
                10
            )
            self.get_logger().info('Successfully subscribed to SportModeState message')
        else:
            self.get_logger().error('unitree_go.msg.SportModeState not available!')
            self.get_logger().error('Please install unitree_go package or check your ROS2 environment')
            return
    
    def sport_mode_state_callback(self, msg):
        """
        处理SportModeState消息的回调函数
        根据C++代码结构转换消息
        """
        try:
            odom_msg = Odometry()
            
            # 设置时间戳 - 使用当前时间，因为SportModeState可能没有时间戳
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.child_frame_id = self.base_frame
            
            # 设置位置 - 从position()数组获取
            # 根据C++代码：position()[0], position()[1], position()[2]
            odom_msg.pose.pose.position.x = float(msg.position[0])
            odom_msg.pose.pose.position.y = float(msg.position[1])
            odom_msg.pose.pose.position.z = float(msg.position[2])
            
            # 设置方向 - 从imu_state.quaternion获取
            # 根据C++代码：quaternion()[0]=w, quaternion()[1]=x, quaternion()[2]=y, quaternion()[3]=z
            odom_msg.pose.pose.orientation.w = float(msg.imu_state.quaternion[0])
            odom_msg.pose.pose.orientation.x = float(msg.imu_state.quaternion[1])
            odom_msg.pose.pose.orientation.y = float(msg.imu_state.quaternion[2])
            odom_msg.pose.pose.orientation.z = float(msg.imu_state.quaternion[3])
            
            # 设置线速度 - 从velocity()数组获取
            # 根据C++代码：velocity()[0], velocity()[1], velocity()[2]
            odom_msg.twist.twist.linear.x = float(msg.velocity[0])
            odom_msg.twist.twist.linear.y = float(msg.velocity[1])
            odom_msg.twist.twist.linear.z = float(msg.velocity[2])
            
            # 设置角速度 - 从yaw_speed()获取
            # 注意：只有yaw_speed，其他角速度设为0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = float(msg.yaw_speed)
            
            # 设置协方差矩阵
            # pose协方差 (6x6 = 36个元素)
            pose_covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            odom_msg.pose.covariance = pose_covariance
            
            # twist协方差 (6x6 = 36个元素)
            twist_covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            odom_msg.twist.covariance = twist_covariance
            
            # 发布odometry消息
            self.odom_publisher.publish(odom_msg)
            
            # 发布TF变换
            if self.publish_tf:
                self.publish_transform(odom_msg)
                
            # 可选：输出调试信息
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                self.get_logger().debug(
                    f'Odom: pos=({msg.position[0]:.3f}, {msg.position[1]:.3f}, {msg.position[2]:.3f}), '
                    f'vel=({msg.velocity[0]:.3f}, {msg.velocity[1]:.3f}, {msg.velocity[2]:.3f}), '
                    f'yaw_speed={msg.yaw_speed:.3f}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing SportModeState message: {e}')
            self.get_logger().error(f'Message type: {type(msg)}')
            
    def publish_transform(self, odom_msg):
        """
        发布TF变换
        """
        try:
            t = TransformStamped()
            
            # 设置时间戳和坐标系
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            
            # 设置平移
            t.transform.translation.x = odom_msg.pose.pose.position.x
            t.transform.translation.y = odom_msg.pose.pose.position.y
            t.transform.translation.z = odom_msg.pose.pose.position.z
            
            # 设置旋转
            t.transform.rotation = odom_msg.pose.pose.orientation
            
            # 发送变换
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    if not UNITREE_MSG_AVAILABLE:
        print("Error: unitree_go package is not available!")
        print("Please install the unitree_go package or check your ROS2 environment.")
        print("You may need to:")
        print("1. Install unitree_go package")
        print("2. Source the workspace containing unitree_go")
        print("3. Check that the package is properly built")
        rclpy.shutdown()
        return
    
    node = OdometryConverterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()