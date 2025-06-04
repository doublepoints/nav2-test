#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTFNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')
        
        # Declare parameters for odom_frame, base_frame and the input topic
        self.odom_frame = self.declare_parameter(
            'odom_frame', 'robot_scan/odom').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'robot_scan/base_link').value
        self.odom_topic = self.declare_parameter(
            'odom_topic', '/robot_scan/odometry').value
        
        self.get_logger().info(f"Publishing TF from '{self.odom_frame}' to '{self.base_frame}'")

        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        t = TransformStamped()

        # Read message content and assign it to corresponding tf variables
        t.header.stamp = msg.header.stamp # Use the timestamp from the Odometry message
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Robot's position in the odom frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z # Typically 0 for 2D robots

        # Robot's orientation in the odom frame (quaternion)
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info(f"Sent transform: {t}") # Uncomment for debugging

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
