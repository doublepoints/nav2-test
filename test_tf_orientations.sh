#!/bin/bash

echo "🔄 检查 TF 方向..."

# 清理进程
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "slam_toolbox" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2

# 设置环境
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "📦 重新构建..."
colcon build --packages-select my_sim_tesi_bringup >/dev/null 2>&1
source install/setup.bash

echo "🚀 启动系统..."
ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
LAUNCH_PID=$!

echo "⏰ 等待系统启动..."
sleep 10

echo "📊 检查 TF 方向..."
echo ""
echo "1. base_link -> livox_frame:"
timeout 2s ros2 run tf2_ros tf2_echo base_link livox_frame 2>/dev/null | grep -A4 "Rotation" | head -5 || echo "等待数据..."

echo ""
echo "2. livox_frame -> livox_frame_corrected:"
timeout 2s ros2 run tf2_ros tf2_echo livox_frame livox_frame_corrected 2>/dev/null | grep -A4 "Rotation" | head -5 || echo "等待数据..."

echo ""
echo "3. base_link -> livox_frame_corrected:"
timeout 2s ros2 run tf2_ros tf2_echo base_link livox_frame_corrected 2>/dev/null | grep -A4 "Rotation" | head -5 || echo "等待数据..."

echo ""
echo "🔍 检查静态变换发布器的参数..."
ros2 node info /static_transform_publisher_laser 2>/dev/null || echo "节点信息不可用"

echo ""
echo "🛑 停止系统..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null || true

echo "✅ 检查完成"