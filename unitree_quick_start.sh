#!/bin/bash

# Unitree Go2 Rosbag建图快速启动脚本
# 使用方法: ./unitree_quick_start.sh /path/to/your/unitree_rosbag

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检查参数
if [ $# -eq 0 ]; then
    echo -e "${RED}错误: 请提供Unitree rosbag路径${NC}"
    echo "使用方法: $0 /path/to/your/unitree_rosbag"
    exit 1
fi

ROSBAG_PATH="$1"

# 检查rosbag文件是否存在
if [ ! -d "$ROSBAG_PATH" ] && [ ! -f "$ROSBAG_PATH" ]; then
    echo -e "${RED}错误: Rosbag路径不存在: $ROSBAG_PATH${NC}"
    exit 1
fi

echo -e "${GREEN}=== Unitree Go2 Rosbag建图系统启动 ===${NC}"
echo -e "${YELLOW}Rosbag路径: $ROSBAG_PATH${NC}"

# 设置工作空间
WORKSPACE_PATH="$HOME/drone_ws"
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo -e "${YELLOW}警告: 工作空间路径不存在，请调整脚本中的WORKSPACE_PATH${NC}"
    echo -e "${YELLOW}当前设置: $WORKSPACE_PATH${NC}"
    read -p "请输入正确的工作空间路径: " NEW_WORKSPACE_PATH
    if [ -d "$NEW_WORKSPACE_PATH" ]; then
        WORKSPACE_PATH="$NEW_WORKSPACE_PATH"
    else
        echo -e "${RED}错误: 工作空间路径仍然不存在${NC}"
        exit 1
    fi
fi

# 进入工作空间
cd "$WORKSPACE_PATH"

# 编译项目
echo -e "${YELLOW}1. 编译项目...${NC}"
colcon build --packages-select my_sim_tesi_ros2_nodes my_sim_tesi_bringup
if [ $? -ne 0 ]; then
    echo -e "${RED}编译失败${NC}"
    exit 1
fi

# 源码设置
source install/setup.bash

# 检查unitree_go包
echo -e "${YELLOW}2. 检查Unitree Go2包依赖...${NC}"
if ros2 pkg list | grep -q unitree_go; then
    echo -e "${GREEN}✓ unitree_go包已找到${NC}"
else
    echo -e "${RED}✗ unitree_go包未找到${NC}"
    echo -e "${YELLOW}请确保已安装unitree_go包或相关的消息定义包${NC}"
    read -p "是否继续? 可能需要手动调整消息类型 (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "取消启动"
        exit 0
    fi
fi

# 检查rosbag信息
echo -e "${YELLOW}3. 分析rosbag内容...${NC}"
echo -e "${BLUE}Rosbag信息:${NC}"
ros2 bag info "$ROSBAG_PATH"

echo
echo -e "${YELLOW}检查关键topics...${NC}"
echo -e "${BLUE}预期的topics:${NC}"
echo "  ✓ /lf/odommodestate (SportModeState消息)"
echo "  ✓ /g1/laserscan (LaserScan消息)"
echo "  ✓ /clock (Clock消息)"

# 快速验证topics
echo -e "${YELLOW}4. 验证消息类型...${NC}"
echo "正在播放rosbag进行快速检查..."

# 在后台播放rosbag进行验证
timeout 10s ros2 bag play "$ROSBAG_PATH" --clock >/dev/null 2>&1 &
ROSBAG_PID=$!

sleep 3

# 检查topics是否存在
if timeout 5s ros2 topic list | grep -q "/lf/odommodestate"; then
    echo -e "${GREEN}✓ 找到odometry topic: /lf/odommodestate${NC}"
    
    # 检查消息类型
    MSG_TYPE=$(timeout 3s ros2 topic info /lf/odommodestate | grep "Type:" | awk '{print $2}')
    if [ "$MSG_TYPE" = "unitree_go/msg/SportModeState" ]; then
        echo -e "${GREEN}✓ 消息类型正确: $MSG_TYPE${NC}"
    else
        echo -e "${YELLOW}⚠ 消息类型: $MSG_TYPE (可能需要调整)${NC}"
    fi
else
    echo -e "${RED}✗ 未找到 /lf/odommodestate topic${NC}"
fi

if timeout 5s ros2 topic list | grep -q "/g1/laserscan"; then
    echo -e "${GREEN}✓ 找到laser topic: /g1/laserscan${NC}"
else
    echo -e "${RED}✗ 未找到 /g1/laserscan topic${NC}"
fi

# 停止后台rosbag
kill $ROSBAG_PID 2>/dev/null || true
wait $ROSBAG_PID 2>/dev/null || true

# 询问是否继续
echo
read -p "是否继续启动建图系统? (y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "取消启动"
    exit 0
fi

# 启动建图系统
echo -e "${YELLOW}5. 启动Unitree Go2建图系统...${NC}"
echo -e "${GREEN}启动命令: ros2 launch my_sim_tesi_bringup unitree_bag_mapping.launch.py bag_path:=$ROSBAG_PATH${NC}"

# 创建启动脚本
LAUNCH_SCRIPT="/tmp/unitree_mapping_launch.sh"
cat > "$LAUNCH_SCRIPT" << EOF
#!/bin/bash
cd $WORKSPACE_PATH
source install/setup.bash
echo "启动Unitree Go2建图系统..."
echo "Rosbag: $ROSBAG_PATH"
echo "按Ctrl+C停止系统"
echo
ros2 launch my_sim_tesi_bringup unitree_bag_mapping.launch.py bag_path:='$ROSBAG_PATH'
EOF

chmod +x "$LAUNCH_SCRIPT"

# 在新终端中启动
if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="Unitree Go2建图系统" -- "$LAUNCH_SCRIPT"
elif command -v xterm >/dev/null 2>&1; then
    xterm -title "Unitree Go2建图系统" -e "$LAUNCH_SCRIPT" &
else
    echo -e "${YELLOW}无法找到终端程序，手动运行:${NC}"
    echo "$LAUNCH_SCRIPT"
    exit 0
fi

echo -e "${GREEN}=== 启动完成 ===${NC}"
echo
echo -e "${BLUE}=== 系统监控和调试命令 ===${NC}"
echo -e "${YELLOW}实时监控:${NC}"
echo "ros2 topic hz /odom                 # 检查odometry频率"
echo "ros2 topic hz /g1/laserscan         # 检查激光频率"
echo "ros2 topic hz /map                  # 检查地图更新"
echo
echo -e "${YELLOW}数据检查:${NC}"
echo "ros2 topic echo /odom --once        # 查看转换后的odometry"
echo "ros2 topic echo /lf/odommodestate --once  # 查看原始odometry"
echo "ros2 topic echo /g1/laserscan --once      # 查看激光数据"
echo
echo -e "${YELLOW}系统状态:${NC}"
echo "ros2 node list                      # 查看运行的节点"
echo "ros2 run tf2_tools view_frames      # 查看TF树"
echo "ros2 node info /slam_toolbox        # 查看SLAM节点状态"
echo
echo -e "${YELLOW}地图保存:${NC}"
echo "ros2 run nav2_map_server map_saver_cli -f unitree_map"
echo
echo -e "${BLUE}如果遇到问题，请检查:${NC}"
echo "1. unitree_go包是否正确安装"
echo "2. 消息类型是否匹配"
echo "3. TF变换是否正确"
echo "4. 激光数据质量是否良好"

# 提供一键停止功能
echo
echo -e "${RED}停止系统: 在终端中按 Ctrl+C${NC}"