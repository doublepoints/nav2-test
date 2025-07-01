#!/bin/bash

echo "🔧 测试 TF 修复..."

# 设置环境
cd /home/tridot/drone_ws_ori2
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH
export ROS_DOMAIN_ID=13
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

echo "📦 构建修改后的包..."
colcon build --packages-select my_sim_tesi_ros2_nodes --symlink-install

if [ $? -eq 0 ]; then
    echo "✅ 构建成功"
    
    # 重新加载环境
    source install/setup.bash
    
    echo "🚀 启动 SLAM 系统..."
    # 在后台启动 launch 文件
    ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py &
    LAUNCH_PID=$!
    
    echo "⏰ 等待系统启动..."
    sleep 10
    
    echo "🎬 启动 rosbag 播放（模拟）..."
    # 这里需要用户提供实际的 rosbag 路径
    echo "请在另一个终端运行："
    echo "ros2 bag play /path/to/your/rosbag --clock"
    
    echo "📊 检查 TF 变换..."
    sleep 5
    
    # 检查 TF 关系
    echo "odom -> base_link 变换："
    timeout 5s ros2 run tf2_ros tf2_echo odom base_link || echo "未检测到 TF 数据"
    
    # 清理
    echo "🛑 停止 launch 进程..."
    kill $LAUNCH_PID 2>/dev/null
    
else
    echo "❌ 构建失败"
fi

echo "✅ 测试完成"