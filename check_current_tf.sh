#!/bin/bash

echo "🔍 检查当前坐标系状态..."

# 清理进程
pkill -f "ros2 launch" 2>/dev/null || true
sleep 1

# 设置环境
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "🚀 启动系统..."
timeout 20s ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
LAUNCH_PID=$!

echo "⏰ 等待系统启动..."
sleep 8

echo "📊 检查各坐标系的具体变换关系..."
echo ""
echo "1️⃣ base_link -> livox_frame:"
timeout 3s ros2 run tf2_ros tf2_echo base_link livox_frame 2>/dev/null | grep -E "(Translation|Rotation)" || echo "等待数据..."

echo ""
echo "2️⃣ livox_frame -> livox_frame_corrected:"
timeout 3s ros2 run tf2_ros tf2_echo livox_frame livox_frame_corrected 2>/dev/null | grep -E "(Translation|Rotation)" || echo "等待数据..."

echo ""
echo "3️⃣ base_link -> livox_frame_corrected (完整路径):"
timeout 3s ros2 run tf2_ros tf2_echo base_link livox_frame_corrected 2>/dev/null | grep -E "(Translation|Rotation)" || echo "等待数据..."

echo ""
echo "🔧 检查静态变换发布器的参数..."
echo "static_transform_publisher_laser 参数:"
ps aux | grep static_transform_publisher | grep laser | head -1

echo ""
echo "livox_frame_corrected_tf 参数:"
ps aux | grep livox_frame_corrected_tf | head -1

echo ""
echo "🛑 停止系统..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null || true

echo "✅ 检查完成"