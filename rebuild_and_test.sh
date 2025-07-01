#!/bin/bash

echo "🔄 重新构建并测试 TF 修复..."

# 设置环境并构建
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "📦 重新构建包..."
colcon build --packages-select my_sim_tesi_ros2_nodes --symlink-install

if [ $? -eq 0 ]; then
    echo "✅ 构建成功"
    
    # 重新加载环境
    source install/setup.bash
    
    echo "🚀 启动系统..."
    ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
    LAUNCH_PID=$!
    
    echo "⏰ 等待系统启动..."
    sleep 8
    
    echo "📊 检查 TF 变换（应该是 0,0,0）..."
    timeout 5s ros2 run tf2_ros tf2_echo odom base_link | head -5
    
    echo ""
    echo "📊 检查 odometry 话题..."
    timeout 3s ros2 topic echo /g1/odometry --once | grep -A3 "position:"
    
    # 清理
    kill $LAUNCH_PID 2>/dev/null
    sleep 2
    
else
    echo "❌ 构建失败"
fi

echo "✅ 测试完成"