#include "my_sim_tesi_ros2_nodes/livox_pointcloud_to_laserscan.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <algorithm>

namespace my_sim_tesi_ros2_nodes
{

LivoxPointCloudToLaserScanNode::LivoxPointCloudToLaserScanNode(const rclcpp::NodeOptions &options)
    : Node("livox_pointcloud_to_laserscan_node", options)
{
    // 声明参数
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("transform_tolerance", 0.01);
    this->declare_parameter("min_height", -0.5);
    this->declare_parameter("max_height", 1.0);
    this->declare_parameter("angle_min", -M_PI);         // -180度
    this->declare_parameter("angle_max", M_PI);          // 180度
    this->declare_parameter("angle_increment", 0.0044);  // 0.25度
    this->declare_parameter("scan_time", 0.033);         // 30Hz
    this->declare_parameter("range_min", 0.2);
    this->declare_parameter("range_max", 20.0);
    this->declare_parameter("use_inf", true);
    this->declare_parameter("inf_epsilon", 1.0);
    this->declare_parameter("use_sim_time", true);

    // 获取参数
    target_frame_ = this->get_parameter("target_frame").as_string();
    transform_tolerance_ = this->get_parameter("transform_tolerance").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();
    scan_time_ = this->get_parameter("scan_time").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    use_inf_ = this->get_parameter("use_inf").as_bool();
    inf_epsilon_ = this->get_parameter("inf_epsilon").as_double();

    // 计算角度范围内的数量
    output_ranges_size_ = std::ceil((angle_max_ - angle_min_) / angle_increment_);

    // 初始化TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建订阅者
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 
        rclcpp::SensorDataQoS(),
        std::bind(&LivoxPointCloudToLaserScanNode::pointcloud_callback, this, std::placeholders::_1)
    );

    // 创建发布者，使用与原始代码相同的QoS设置
    auto qos = rclcpp::QoS(1)
               .reliability(RMW_QOS_RELIABILITY_POLICY_BEST_EFFORT)
               .durability(RMW_QOS_DURABILITY_POLICY_VOLATILE)
               .history(RMW_QOS_HISTORY_POLICY_KEEP_LAST);
    
    laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/g1/laserscan", qos);

    RCLCPP_INFO(this->get_logger(), "Livox PointCloud to LaserScan converter node started");
    RCLCPP_INFO(this->get_logger(), "Input topic: /livox/lidar");
    RCLCPP_INFO(this->get_logger(), "Output topic: /g1/laserscan");
    RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
}

void LivoxPointCloudToLaserScanNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    try {
        // 创建LaserScan消息
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        
        // 设置LaserScan头部信息
        scan_msg->header = cloud_msg->header;
        scan_msg->header.frame_id = target_frame_;
        
        // 设置扫描参数
        scan_msg->angle_min = angle_min_;
        scan_msg->angle_max = angle_max_;
        scan_msg->angle_increment = angle_increment_;
        scan_msg->time_increment = 0.0;
        scan_msg->scan_time = scan_time_;
        scan_msg->range_min = range_min_;
        scan_msg->range_max = range_max_;

        // 初始化ranges数组
        scan_msg->ranges.assign(output_ranges_size_, use_inf_ ? std::numeric_limits<float>::infinity() : range_max_);

        // 如果需要变换到目标坐标系
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        if (cloud_msg->header.frame_id != target_frame_) {
            if (!transform_pointcloud(*cloud_msg, transformed_cloud)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Failed to transform point cloud from %s to %s",
                                   cloud_msg->header.frame_id.c_str(), target_frame_.c_str());
                return;
            }
        } else {
            transformed_cloud = *cloud_msg;
        }

        // 处理点云数据
        process_pointcloud(transformed_cloud, *scan_msg);

        // 发布LaserScan消息
        laserscan_pub_->publish(*scan_msg);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

bool LivoxPointCloudToLaserScanNode::transform_pointcloud(const sensor_msgs::msg::PointCloud2& cloud_in, 
                                                         sensor_msgs::msg::PointCloud2& cloud_out)
{
    try {
        // 查找变换
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            target_frame_, 
            cloud_in.header.frame_id,
            cloud_in.header.stamp,
            rclcpp::Duration::from_seconds(transform_tolerance_)
        );

        // 变换点云
        tf2::doTransform(cloud_in, cloud_out, transform);
        return true;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Could not transform %s to %s: %s",
                            cloud_in.header.frame_id.c_str(), 
                            target_frame_.c_str(), 
                            ex.what());
        return false;
    }
}

void LivoxPointCloudToLaserScanNode::process_pointcloud(const sensor_msgs::msg::PointCloud2& cloud, 
                                                       sensor_msgs::msg::LaserScan& scan)
{
    // 创建点云迭代器
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    // 遍历所有点
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;

        // 检查点是否有效
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        // 高度过滤
        if (z < min_height_ || z > max_height_) {
            continue;
        }

        // 计算距离和角度
        float range = std::sqrt(x * x + y * y);
        
        // 距离过滤
        if (range < range_min_ || range > range_max_) {
            continue;
        }

        // 计算角度
        float angle = std::atan2(y, x);

        // 确保角度在范围内
        if (angle < angle_min_ || angle > angle_max_) {
            continue;
        }

        // 计算角度索引
        int index = static_cast<int>((angle - angle_min_) / angle_increment_);
        
        // 确保索引在有效范围内
        if (index >= 0 && index < static_cast<int>(scan.ranges.size())) {
            // 如果当前距离更近，则更新
            if (range < scan.ranges[index] || 
                (use_inf_ && std::isinf(scan.ranges[index]))) {
                scan.ranges[index] = range;
            }
        }
    }

    // 处理无穷值
    if (!use_inf_) {
        for (auto& range : scan.ranges) {
            if (std::isinf(range)) {
                range = range_max_;
            }
        }
    }
}

} // namespace my_sim_tesi_ros2_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(my_sim_tesi_ros2_nodes::LivoxPointCloudToLaserScanNode)
