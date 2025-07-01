#!/bin/bash

echo "🔍 检查 odom 和 base_link 之间的 X,Y 误差..."

# 设置环境
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "🚀 启动系统..."
ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
LAUNCH_PID=$!

echo "⏰ 等待系统启动..."
sleep 8

echo "📊 检查初始 TF 变换（应该接近 0,0,0）..."
timeout 3s ros2 run tf2_ros tf2_echo odom base_link | head -10

echo ""
echo "💡 如果看到较大的 X,Y 值，说明还有问题需要修复"

# 清理
kill $LAUNCH_PID 2>/dev/null
sleep 2

echo "✅ 检查完成"