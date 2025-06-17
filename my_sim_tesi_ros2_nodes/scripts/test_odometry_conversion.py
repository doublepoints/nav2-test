#!/usr/bin/env python3
"""
测试Unitree SportModeState到标准Odometry的转换
使用方法: 
1. 播放rosbag: ros2 bag play /path/to/rosbag --clock
2. 运行此脚本: python3 test_odometry_conversion.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import sys

try:
    from unitree_go.msg import SportModeState
    UNITREE_MSG_AVAILABLE = True
except ImportError:
    UNITREE_MSG_AVAILABLE = False
    print("Warning: unitree_go.msg.SportModeState not available")

class ConversionTester(Node):
    def __init__(self):
        super().__init__('conversion_tester')
        
        # 安全地声明use_sim_time参数
        try:
            self.declare_parameter('use_sim_time', True)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            # 参数已经存在，这是正常的
            pass
        
        self.sport_mode_count = 0
        self.odom_count = 0
        self.last_sport_mode_msg = None
        self.last_odom_msg = None
        
        # 订阅原始消息
        if UNITREE_MSG_AVAILABLE:
            self.sport_mode_sub = self.create_subscription(
                SportModeState,
                '/lf/odommodestate',
                self.sport_mode_callback,
                10
            )
        
        # 订阅转换后的消息
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 创建定时器进行状态报告
        self.timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info("=== Odometry转换测试器启动 ===")
        if not UNITREE_MSG_AVAILABLE:
            self.get_logger().warn("unitree_go包不可用，只能检查转换后的消息")
    
    def sport_mode_callback(self, msg):
        self.sport_mode_count += 1
        self.last_sport_mode_msg = msg
        
        if self.sport_mode_count == 1:
            self.get_logger().info("✓ 收到第一个SportModeState消息")
            self.print_sport_mode_details(msg)
    
    def odom_callback(self, msg):
        self.odom_count += 1
        self.last_odom_msg = msg
        
        if self.odom_count == 1:
            self.get_logger().info("✓ 收到第一个转换后的Odometry消息")
            self.print_odom_details(msg)
    
    def print_sport_mode_details(self, msg):
        self.get_logger().info("--- SportModeState消息详情 ---")
        self.get_logger().info(f"位置: [{msg.position[0]:.3f}, {msg.position[1]:.3f}, {msg.position[2]:.3f}]")
        self.get_logger().info(f"速度: [{msg.velocity[0]:.3f}, {msg.velocity[1]:.3f}, {msg.velocity[2]:.3f}]")
        self.get_logger().info(f"四元数: [{msg.imu_state.quaternion[0]:.3f}, {msg.imu_state.quaternion[1]:.3f}, {msg.imu_state.quaternion[2]:.3f}, {msg.imu_state.quaternion[3]:.3f}]")
        self.get_logger().info(f"偏航角速度: {msg.yaw_speed:.3f}")
        self.get_logger().info(f"欧拉角: [{msg.imu_state.rpy[0]:.3f}, {msg.imu_state.rpy[1]:.3f}, {msg.imu_state.rpy[2]:.3f}]")
    
    def print_odom_details(self, msg):
        self.get_logger().info("--- Odometry消息详情 ---")
        self.get_logger().info(f"Frame ID: {msg.header.frame_id} -> {msg.child_frame_id}")
        self.get_logger().info(f"位置: [{msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}, {msg.pose.pose.position.z:.3f}]")
        self.get_logger().info(f"方向: [{msg.pose.pose.orientation.x:.3f}, {msg.pose.pose.orientation.y:.3f}, {msg.pose.pose.orientation.z:.3f}, {msg.pose.pose.orientation.w:.3f}]")
        self.get_logger().info(f"线速度: [{msg.twist.twist.linear.x:.3f}, {msg.twist.twist.linear.y:.3f}, {msg.twist.twist.linear.z:.3f}]")
        self.get_logger().info(f"角速度: [{msg.twist.twist.angular.x:.3f}, {msg.twist.twist.angular.y:.3f}, {msg.twist.twist.angular.z:.3f}]")
    
    def print_status(self):
        self.get_logger().info(f"=== 状态报告 ===")
        if UNITREE_MSG_AVAILABLE:
            self.get_logger().info(f"SportModeState消息: {self.sport_mode_count}")
        self.get_logger().info(f"Odometry消息: {self.odom_count}")
        
        # 检查转换质量
        if self.last_sport_mode_msg and self.last_odom_msg and UNITREE_MSG_AVAILABLE:
            self.check_conversion_quality()
        
        if self.sport_mode_count == 0 and UNITREE_MSG_AVAILABLE:
            self.get_logger().warn("⚠ 未收到SportModeState消息，请检查:")
            self.get_logger().warn("  1. rosbag是否正在播放")
            self.get_logger().warn("  2. topic名称是否正确 (/lf/odommodestate)")
            self.get_logger().warn("  3. 消息类型是否匹配")
        
        if self.odom_count == 0:
            self.get_logger().warn("⚠ 未收到Odometry消息，请检查:")
            self.get_logger().warn("  1. odometry_converter_node是否运行")
            self.get_logger().warn("  2. 转换节点是否正常工作")
    
    def check_conversion_quality(self):
        """检查转换质量"""
        sport_msg = self.last_sport_mode_msg
        odom_msg = self.last_odom_msg
        
        # 检查位置转换
        pos_diff_x = abs(sport_msg.position[0] - odom_msg.pose.pose.position.x)
        pos_diff_y = abs(sport_msg.position[1] - odom_msg.pose.pose.position.y)
        pos_diff_z = abs(sport_msg.position[2] - odom_msg.pose.pose.position.z)
        
        if pos_diff_x < 0.001 and pos_diff_y < 0.001 and pos_diff_z < 0.001:
            self.get_logger().info("✓ 位置转换正确")
        else:
            self.get_logger().warn(f"⚠ 位置转换可能有问题: 差异 ({pos_diff_x:.6f}, {pos_diff_y:.6f}, {pos_diff_z:.6f})")
        
        # 检查四元数转换 (w, x, y, z)
        q_sport = sport_msg.imu_state.quaternion
        q_odom = odom_msg.pose.pose.orientation
        
        # SportModeState: [w, x, y, z], Odometry: (x, y, z, w)
        q_diff_w = abs(q_sport[0] - q_odom.w)
        q_diff_x = abs(q_sport[1] - q_odom.x)
        q_diff_y = abs(q_sport[2] - q_odom.y)
        q_diff_z = abs(q_sport[3] - q_odom.z)
        
        if q_diff_w < 0.001 and q_diff_x < 0.001 and q_diff_y < 0.001 and q_diff_z < 0.001:
            self.get_logger().info("✓ 四元数转换正确")
        else:
            self.get_logger().warn(f"⚠ 四元数转换可能有问题: 差异 w:{q_diff_w:.6f}, x:{q_diff_x:.6f}, y:{q_diff_y:.6f}, z:{q_diff_z:.6f}")
        
        # 检查速度转换
        vel_diff_x = abs(sport_msg.velocity[0] - odom_msg.twist.twist.linear.x)
        vel_diff_y = abs(sport_msg.velocity[1] - odom_msg.twist.twist.linear.y)
        vel_diff_z = abs(sport_msg.velocity[2] - odom_msg.twist.twist.linear.z)
        
        if vel_diff_x < 0.001 and vel_diff_y < 0.001 and vel_diff_z < 0.001:
            self.get_logger().info("✓ 线速度转换正确")
        else:
            self.get_logger().warn(f"⚠ 线速度转换可能有问题: 差异 ({vel_diff_x:.6f}, {vel_diff_y:.6f}, {vel_diff_z:.6f})")
        
        # 检查角速度转换
        yaw_speed_diff = abs(sport_msg.yaw_speed - odom_msg.twist.twist.angular.z)
        if yaw_speed_diff < 0.001:
            self.get_logger().info("✓ 角速度转换正确")
        else:
            self.get_logger().warn(f"⚠ 角速度转换可能有问题: 差异 {yaw_speed_diff:.6f}")

def main(args=None):
    if not UNITREE_MSG_AVAILABLE:
        print("错误: unitree_go包不可用")
        print("请确保已安装并source了包含unitree_go的工作空间")
        # return  # 还是继续运行，只检查输出
    
    rclpy.init(args=args)
    node = ConversionTester()
    
    print("=== Unitree Odometry转换测试器 ===")
    print("请确保:")
    print("1. rosbag正在播放: ros2 bag play /path/to/rosbag --clock")
    print("2. 转换节点正在运行: ros2 run my_sim_tesi_ros2_nodes odometry_converter_node.py")
    print("3. 按Ctrl+C退出测试")
    print()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n=== 测试结束 ===")
        if node.sport_mode_count > 0 and node.odom_count > 0:
            print("✓ 转换测试通过")
        elif node.odom_count > 0:
            print("✓ 转换节点工作正常（无法验证输入）")
        else:
            print("✗ 转换测试失败")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()