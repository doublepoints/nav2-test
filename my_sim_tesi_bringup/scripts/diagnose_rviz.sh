#!/bin/bash

echo "🔍 RViz地图显示诊断..."

# 1. 检查地图话题
echo "📊 检查地图话题..."
ros2 topic hz /map &
TOPIC_PID=$!
sleep 3
kill $TOPIC_PID 2>/dev/null

echo ""
echo "🗺️ 检查地图内容..."
ros2 topic echo /map --once | head -10

echo ""
echo "⚙️ 检查QoS设置..."
ros2 topic info /map --verbose | grep -A 15 "Publisher count"

echo ""
echo "🎯 测试地图订阅..."
python3 /home/tridot/drone_ws_ori2/src/my_sim_tesi_bringup/scripts/test_rviz_map.py &
TEST_PID=$!
sleep 5
kill $TEST_PID 2>/dev/null

echo ""
echo "✅ 诊断完成!"
echo ""
echo "🚀 现在启动简化版RViz..."
rviz2 -d src/my_sim_tesi_bringup/config/simple_slam_viz.rviz