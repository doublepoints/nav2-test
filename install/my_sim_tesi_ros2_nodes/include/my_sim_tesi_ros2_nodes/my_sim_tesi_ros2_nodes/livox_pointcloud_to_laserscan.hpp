#ifndef MY_SIM_TESI_ROS2_NODES_LIVOX_POINTCLOUD_TO_LASERSCAN_HPP_
#define MY_SIM_TESI_ROS2_NODES_LIVOX_POINTCLOUD_TO_LASERSCAN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <memory>

namespace my_sim_tesi_ros2_nodes
{

/**
 * @class LivoxPointCloudToLaserScanNode
 * @brief 将Livox雷达的PointCloud2数据转换为LaserScan格式
 * 
 * 该节点订阅/livox/lidar话题的PointCloud2消息，将其转换为LaserScan格式
 * 并发布到/g1/laserscan话题。支持坐标系变换和各种过滤参数配置。
 */
class LivoxPointCloudToLaserScanNode : public rclcpp::Node
{
public:
    explicit LivoxPointCloudToLaserScanNode(const rclcpp::NodeOptions &options);

private:
    // 回调函数
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    
    // 辅助函数
    bool transform_pointcloud(const sensor_msgs::msg::PointCloud2& cloud_in, 
                             sensor_msgs::msg::PointCloud2& cloud_out);
    void process_pointcloud(const sensor_msgs::msg::PointCloud2& cloud, 
                           sensor_msgs::msg::LaserScan& scan);

    // ROS通信对象
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
    
    // TF相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 配置参数
    std::string target_frame_;
    double transform_tolerance_;
    double min_height_;
    double max_height_;
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double scan_time_;
    double range_min_;
    double range_max_;
    bool use_inf_;
    double inf_epsilon_;
    
    size_t output_ranges_size_;
};

} // namespace my_sim_tesi_ros2_nodes

#endif // MY_SIM_TESI_ROS2_NODES_LIVOX_POINTCLOUD_TO_LASERSCAN_HPP_
