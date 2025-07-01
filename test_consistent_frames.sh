#!/bin/bash

echo "🔧 测试坐标系一致性方案..."

# 清理进程
pkill -f "ros2 launch" 2>/dev/null || true
sleep 2

# 设置环境
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "📦 重新构建..."
colcon build --packages-select my_sim_tesi_bringup >/dev/null 2>&1
source install/setup.bash

echo "🚀 启动系统..."
timeout 20s ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
LAUNCH_PID=$!

echo "⏰ 等待系统启动..."
sleep 10

echo "📊 检查坐标系一致性..."
echo ""
echo "1️⃣ base_link -> livox_frame (应该是 0° 旋转):"
timeout 3s ros2 run tf2_ros tf2_echo base_link livox_frame 2>/dev/null | grep -E "RPY.*degree" || echo "等待数据..."

echo ""
echo "2️⃣ livox_frame -> livox_frame_corrected (应该是 0° 旋转):"
timeout 3s ros2 run tf2_ros tf2_echo livox_frame livox_frame_corrected 2>/dev/null | grep -E "RPY.*degree" || echo "等待数据..."

echo ""
echo "3️⃣ base_link -> livox_frame_corrected (应该是 0° 旋转):"
timeout 3s ros2 run tf2_ros tf2_echo base_link livox_frame_corrected 2>/dev/null | grep -E "RPY.*degree" || echo "等待数据..."

echo ""
echo "💡 预期结果："
echo "- 所有坐标系应该有相同的方向 (0° 旋转)"
echo "- 数据修正已经在 ground_segmentation_node 代码中完成"
echo "- livox_frame_corrected 应该与 base_link 方向一致"

echo ""
echo "🛑 停止系统..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null || true

echo "✅ 测试完成 - 现在所有坐标系应该方向一致"