#!/bin/bash

echo "🔍 验证 TF 链条..."

# 清理进程
pkill -f "ros2 launch" 2>/dev/null || true
sleep 1

# 设置环境
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "🚀 启动系统..."
timeout 15s ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
LAUNCH_PID=$!

echo "⏰ 等待系统启动..."
sleep 8

echo "📊 检查完整 TF 链..."
echo ""
echo "❓ base_link -> livox_frame_corrected (完整路径):"
timeout 3s ros2 run tf2_ros tf2_echo base_link livox_frame_corrected 2>/dev/null | grep -A4 "Rotation" | head -5 || echo "等待数据..."

echo ""
echo "🔗 TF 树结构:"
timeout 3s ros2 run tf2_tools view_frames.py 2>/dev/null || echo "无法生成 TF 树"

echo ""
echo "🛑 停止系统..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null || true

echo "✅ 验证完成"