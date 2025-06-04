#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
# import tf_transformations # 一般不需要显式导入来做四元数赋值

class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')

        # 声明参数，允许从launch文件或命令行覆盖
        self.declare_parameter('odom_topic', '/robot_scan/odometry')
        self.declare_parameter('publish_tf', True)
        # 新增参数用于指定期望的 odom 和 base_link frame ID
        self.declare_parameter('expected_odom_frame', 'robot_scan/odom')
        self.declare_parameter('expected_base_frame', 'robot_scan/base_link')

        self.publish_tf_ = self.get_parameter('publish_tf').get_parameter_value().bool_value
        odom_topic_name = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.expected_odom_frame_ = self.get_parameter('expected_odom_frame').get_parameter_value().string_value
        self.expected_base_frame_ = self.get_parameter('expected_base_frame').get_parameter_value().string_value

        if not self.publish_tf_:
            self.get_logger().info(f'TF publishing is disabled for {self.get_name()}.')
            return

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic_name,
            self.odom_callback,
            10) # QoS Profile 10 for sensor data is a common default
        self.get_logger().info(f'Odometry to TF broadcaster started for topic: {odom_topic_name}')
        self.get_logger().info(f'Publishing TF: {self.expected_odom_frame_} -> {self.expected_base_frame_}')


    def odom_callback(self, msg: Odometry):
        if not self.publish_tf_:
            return

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        
        # 使用配置的 frame ID，而不是直接从消息中读取
        # 确保 header.frame_id (odom frame) 是正确的
        if msg.header.frame_id == self.expected_odom_frame_:
            t.header.frame_id = self.expected_odom_frame_
        elif msg.header.frame_id == "odom" and self.expected_odom_frame_.endswith("/odom"): # 处理Gazebo可能只发odom的情况
             t.header.frame_id = self.expected_odom_frame_
             self.get_logger().debug(f"Odometry header.frame_id is '{msg.header.frame_id}', using configured '{self.expected_odom_frame_}'.")
        else:
            self.get_logger().warn(
                f"Odometry message header.frame_id is '{msg.header.frame_id}', "
                f"but expected '{self.expected_odom_frame_}'. Using expected value. "
                f"Consider checking Gazebo odometry plugin frame configuration."
            )
            t.header.frame_id = self.expected_odom_frame_


        # 确保 child_frame_id (base_link frame) 是正确的
        t.child_frame_id = self.expected_base_frame_

        # 从里程计消息中读取时间戳和坐标系名称
        # 确保使用里程计消息的时间戳，这对时间同步至关重要
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id      # 应该是 'robot_scan/odom'
        t.child_frame_id = msg.child_frame_id      # 应该是 'robot_scan/base_link' (根据SDF修改)

        # 设置变换的平移
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # 设置变换的旋转 (从四元数)
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # 发布变换
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    if node.publish_tf_: # 只有在需要发布TF时才spin
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
    else: # 如果不发布TF，节点可以做其他事或直接退出
        node.get_logger().info(f'{node.get_name()} initialized but TF publishing is disabled. Shutting down if not spinning.')
        node.destroy_node() # 如果不spin，确保销毁

    rclpy.shutdown()

if __name__ == '__main__':
    main()