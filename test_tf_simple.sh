#!/bin/bash

echo "🧹 首先清理所有进程..."
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "slam_toolbox" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2

echo "🔄 设置环境..."
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "📦 确保最新构建..."
colcon build --packages-select my_sim_tesi_ros2_nodes --symlink-install >/dev/null 2>&1
source install/setup.bash

echo "🚀 启动系统（单一实例）..."
ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
LAUNCH_PID=$!

echo "⏰ 等待系统完全启动..."
sleep 10

echo "📊 检查 TF 变换..."
echo "应该看到接近 (0,0,0) 的值："
timeout 3s ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null || echo "等待 TF 数据..."

echo ""
echo "🛑 停止系统..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null || true
sleep 2

echo "✅ 测试完成"