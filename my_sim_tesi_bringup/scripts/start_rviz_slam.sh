#!/bin/bash

# SLAM可视化启动脚本
echo "🚀 启动SLAM可视化..."

# 设置ROS环境
source install/setup.bash
export ROS_DOMAIN_ID=13

# 启动RViz2
echo "📊 启动RViz2进行SLAM可视化..."
rviz2 -d src/my_sim_tesi_bringup/config/slam_visualization.rviz

echo "✅ RViz2已启动完成！"