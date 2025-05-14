#ifndef MY_SIM_TESI_ROS2_NODES_SIMPLE_CONTROLLER_HPP_
#define MY_SIM_TESI_ROS2_NODES_SIMPLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"  // 使用服务而不是消息
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

namespace my_sim_tesi_ros2_nodes {

class SimpleController : public rclcpp::Node {
public:
    SimpleController(const rclcpp::NodeOptions &options);

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void update_pose();
    void update_model_state();
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;  // 改为服务客户端
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    
    std::string model_name_;
    std::string world_name_;
    double current_x_;
    double current_y_;
    double current_z_;
    double current_yaw_;
    
    double linear_x_;
    double angular_z_;
    rclcpp::Time last_update_time_;
    bool first_update_;
};

}  // namespace my_sim_tesi_ros2_nodes
#endif  // MY_SIM_TESI_ROS2_NODES_SIMPLE_CONTROLLER_HPP_