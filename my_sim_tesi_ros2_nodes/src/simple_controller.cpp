#include "my_sim_tesi_ros2_nodes/simple_controller.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace my_sim_tesi_ros2_nodes {

SimpleController::SimpleController(const rclcpp::NodeOptions &options)
: Node("simple_controller", options), 
  current_x_(0.0), current_y_(0.0), current_z_(0.16), current_yaw_(0.0),
  linear_x_(0.0), angular_z_(0.0), first_update_(true)
{
    using namespace std::placeholders;
    
    this->declare_parameter<std::string>("model_name", "robot_scan");
    this->declare_parameter<double>("initial_x", -13.104900);
    this->declare_parameter<double>("initial_y", 2.635770);
    this->declare_parameter<double>("initial_yaw", -1.602640);
    this->declare_parameter<double>("update_rate", 50.0); // Hz
    
    model_name_ = this->get_parameter("model_name").as_string();
    current_x_ = this->get_parameter("initial_x").as_double();
    current_y_ = this->get_parameter("initial_y").as_double();
    current_yaw_ = this->get_parameter("initial_yaw").as_double();
    double update_rate = this->get_parameter("update_rate").as_double();
    
    // 创建订阅者，监听Nav2发出的cmd_vel命令
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_smoothed", 10, std::bind(&SimpleController::cmd_vel_callback, this, _1));
    
    // 使用话题发布者替代服务客户端
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/robot_scan/pose", 10);

    // 创建里程计发布者
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot_scan/odometry", 10);
    
    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // 创建更新定时器
    auto update_period = std::chrono::duration<double>(1.0 / update_rate);
    update_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(update_period),
        std::bind(&SimpleController::update_pose, this));
    
    last_update_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Simple controller initialized for Ignition Gazebo");
    RCLCPP_INFO(this->get_logger(), "Controlling model: %s", model_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Initial pose: [x=%f, y=%f, yaw=%f]", 
                current_x_, current_y_, current_yaw_);
    // 初始化位置
    update_model_state();
}

void SimpleController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // 存储收到的速度命令
    linear_x_ = msg->linear.x;
    angular_z_ = msg->angular.z;
    // 添加日志，用于调试
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear_x=%f, angular_z=%f", 
                linear_x_, angular_z_);
}

void SimpleController::update_pose() {
    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    
    if (first_update_) {
        first_update_ = false;
        return;
    }
    
    // 更新姿态
    current_yaw_ += angular_z_ * dt;
    
    // 标准化角度到[-π, π]
    while (current_yaw_ > M_PI) current_yaw_ -= 2.0 * M_PI;
    while (current_yaw_ < -M_PI) current_yaw_ += 2.0 * M_PI;
    
    // 更新位置
    current_x_ += linear_x_ * std::cos(current_yaw_) * dt;
    current_y_ += linear_x_ * std::sin(current_yaw_) * dt;
    
    // 更新Gazebo中的模型状态
    update_model_state();
    
    // 发布里程计信息
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "robot_scan/odom";
    odom.child_frame_id = "robot_scan/base_link";
    
    odom.pose.pose.position.x = current_x_;
    odom.pose.pose.position.y = current_y_;
    odom.pose.pose.position.z = current_z_;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, current_yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    odom.twist.twist.linear.x = linear_x_;
    odom.twist.twist.angular.z = angular_z_;
    
    odom_publisher_->publish(odom);
    
    // 发布TF变换
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = "robot_scan/odom";
    transform.child_frame_id = "robot_scan/base_link";
    
    transform.transform.translation.x = current_x_;
    transform.transform.translation.y = current_y_;
    transform.transform.translation.z = current_z_;
    
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transform);
}

void SimpleController::update_model_state() {
    // 创建Pose消息
    geometry_msgs::msg::Pose pose_msg;
    
    // 设置位置
    pose_msg.position.x = current_x_;
    pose_msg.position.y = current_y_;
    pose_msg.position.z = current_z_;
    
    // 设置朝向
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, current_yaw_);
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    
    // 发布Pose消息
    pose_publisher_->publish(pose_msg);
}

}  // namespace my_sim_tesi_ros2_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(my_sim_tesi_ros2_nodes::SimpleController)