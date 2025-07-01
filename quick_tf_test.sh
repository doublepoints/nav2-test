#!/bin/bash

echo "🔧 快速 TF 方向测试..."

# 清理进程
pkill -f "ros2 launch" 2>/dev/null || true
sleep 1

# 设置环境
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "📦 重新构建..."
colcon build --packages-select my_sim_tesi_bringup >/dev/null 2>&1
source install/setup.bash

echo "🚀 启动系统..."
timeout 20s ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
LAUNCH_PID=$!

echo "⏰ 等待系统启动..."
sleep 8

echo "📊 检查 TF 旋转参数..."
echo ""
echo "base_link -> livox_frame 旋转:"
timeout 3s ros2 run tf2_ros tf2_echo base_link livox_frame 2>/dev/null | grep -A4 "Rotation" | head -5 || echo "等待数据..."

echo ""
echo "livox_frame -> livox_frame_corrected 旋转:"
timeout 3s ros2 run tf2_ros tf2_echo livox_frame livox_frame_corrected 2>/dev/null | grep -A4 "Rotation" | head -5 || echo "等待数据..."

echo ""
echo "🛑 停止系统..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null || true

echo "✅ 测试完成"