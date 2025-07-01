#!/bin/bash

echo "🧹 清理所有ROS2相关进程..."

# 终结各种ROS2进程
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true
pkill -f "slam_toolbox" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
pkill -f "ground_segmentation" 2>/dev/null || true
pkill -f "pointcloud_to_laserscan" 2>/dev/null || true
pkill -f "odometry_converter" 2>/dev/null || true
pkill -f "async_slam_toolbox" 2>/dev/null || true
pkill -f "livox_pointcloud_to_laserscan" 2>/dev/null || true
pkill -f "ros2-daemon" 2>/dev/null || true

echo "⏰ 等待进程完全终止..."
sleep 3

echo "🔍 检查剩余进程..."
remaining=$(ps aux | grep -E "(ros2|rviz|slam|static_transform)" | grep -v grep | wc -l)
if [ $remaining -eq 0 ]; then
    echo "✅ 所有相关进程已清理完毕"
else
    echo "⚠️  仍有 $remaining 个相关进程在运行:"
    ps aux | grep -E "(ros2|rviz|slam|static_transform)" | grep -v grep
fi

echo ""
echo "📋 修改总结："
echo "1. ✅ RViz地图显示问题 - 已修复相对路径"
echo "2. ✅ odom/base_link TF问题 - 已修复里程计转换"  
echo "3. ✅ 坐标系方向问题 - 已修复static_transform_publisher参数"
echo ""
echo "🔧 主要修改文件："
echo "- complete_rosbag_slam.launch.py (RViz路径 + TF旋转参数)"
echo "- odometry_converter_node.py (里程计相对偏移计算)"
echo ""
echo "🚀 测试命令："
echo "source install/setup.bash"
echo "source /home/tridot/CProjects/unitree_ros2/setup_default.sh"
echo "ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py"
echo ""
echo "✨ 现在您可以开始测试了！"