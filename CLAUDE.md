# CLAUDE.md

此文件为 Claude Code (claude.ai/code) 在处理此仓库代码时提供指导。

## 构建和开发命令

### 环境设置（执行任何命令前必需）
```bash
# 加载工作空间
source install/setup.bash

# 设置 Gazebo 资源路径（根据需要调整路径）
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share

# 设置库路径
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH

# 设置 ROS 域（避免冲突）
export ROS_DOMAIN_ID=13

# 如果使用 conda 环境，请停用 anaconda
```

### 构建命令
```bash
# 构建整个工作空间（从工作空间根目录运行）
colcon build  --symlink-install

# 构建特定包
colcon build --packages-select <包名>
```

### 主要启动命令

**导航和定位：**
```bash
ros2 launch my_sim_tesi_bringup my_sim_final.launch.py
```

**SLAM 建图：**
```bash
ros2 launch my_sim_tesi_bringup my_sim_map_scan.launch.py
# 在 Gazebo GUI 中，启用 "Key Publisher" 插件以控制机器人
```

**保存地图（SLAM 之后）：**
```bash
ros2 run nav2_map_server map_saver_cli
```

**点云处理（需要 CycloneDDS）：**
```bash
# （所有终端在启动前必须执行）启动CycloneDDS
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

# 终端 1：启动点云到激光扫描的转换
ros2 launch my_sim_tesi_bringup livox_to_laserscan.launch.py

# 终端 2：启动地面分割
ros2 launch linefit_ground_segmentation_ros segmentation.launch.py

# 终端 3：播放 rosbag（如果使用录制的数据）
ros2 bag play /path/to/bag --loop

# 终端 4：可视化
rviz2 -d src/my_sim_tesi_bringup/config/livox_visualization.rviz
```

**建立适用于unitree g1 rosbag2的地图（需要 CycloneDDS）：**
```
# 环境设置，（所有终端在启动前必须执行）启动CycloneDDS
source /home/tridot/CProjects/unitree_ros2/setup_default.sh

# 终端 1：启动建图程序（从点云转换为laserscan,利用laserscan建立地图）
ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py

# 终端 2：播放 rosbag（如果使用录制的数据）
ros2 bag play /path/to/bag --clock

（例如：ros2 bag play /media/tridot/DATA2/dataset2/G1/rosbag2_2025_05_26-15_31_21 --clock）

# 终端 3：显示地图
#方法1：
./src/my_sim_tesi_bringup/scripts/diagnose_rviz.sh

#方法2：
rviz2 -d src/my_sim_tesi_bringup/config/simple_slam_viz.rviz

# 终端 4：保存地图
ros2 run nav2_map_server map_saver_cli

```

## 架构概述

这是一个用于无人机/机器人仿真、建图和导航的 ROS2 工作空间，包含以下关键组件：

### 核心包

**my_sim_tesi_bringup**：主要启动文件和配置
- 包含不同仿真模式的启动文件
- SLAM、导航和传感器处理的配置文件
- 用于可视化的 RViz 配置

**my_sim_tesi_gazebo**：Gazebo 仿真环境
- 自定义世界文件和机器人模型（Pioneer2DX、四旋翼）
- 仿真环境的 3D 模型和纹理
- SDF 模型定义

**my_sim_tesi_ros2_nodes**：自定义 ROS2 节点
- 用于姿态控制的动作服务器/客户端
- 系统协调的协调器节点
- 点云到激光扫描转换节点
- 里程计处理工具

**my_sim_tesi_ros2_interfaces**：自定义消息/动作定义
- DronePoseControl 动作接口

**linefit_ground_segmentation**：地面平面分割
- 从点云中分离地面点和障碍物
- 用于复杂 3D 环境中的导航

### 系统数据流

1. **仿真**：Gazebo 提供传感器数据（激光雷达、里程计、相机）
2. **桥接**：ros_gz_bridge 在 Gazebo 和 ROS2 之间传输数据
3. **处理**：自定义节点处理传感器数据（地面分割、坐标变换）
4. **SLAM/导航**：slam_toolbox 或 nav2 用于建图和导航
5. **可视化**：RViz2 显示地图、机器人状态和传感器数据

### 关键配置文件

- `ros_gz_bridge_*_config.yaml`：定义 Gazebo 和 ROS2 之间的话题桥接
- `slam_toolbox_config.yaml`：SLAM 算法参数
- `nav2_param_config.yaml`：导航栈配置
- `*.rviz`：不同用例的可视化配置

### TF 坐标系结构
```
map → robot_scan/odom → robot_scan/base_footprint → [传感器坐标系]
```

### 重要话题
- `/robot_scan/scan`：用于 SLAM/导航的激光扫描数据
- `/robot_scan/odometry`：机器人里程计
- `/map`：SLAM 输出的占据栅格地图
- `/tf`：动态坐标变换
- `/robot_scan/cmd_vel`：机器人控制的速度命令

## 开发说明

- 此工作空间支持地面机器人（Pioneer2DX）和无人机（四旋翼）仿真
- 系统可以处理 2D 激光扫描和 3D 点云
- 地面分割对于 3D 点云处理尤其重要
- 通过不同的启动文件支持多种仿真场景
- 使用 Ignition Gazebo（而非经典 Gazebo）

## 修改记录

### 2025-07-01 修改记录

**问题1（已解决）**：运行 `ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py` 建图后，无法在 RViz 中显示地图。

**解决方案**：
- 问题原因：launch 文件中使用绝对路径加载 RViz 配置文件导致地图无法显示
- 修改内容：将 `os.path.join(pkg_project_bringup, 'config', 'simple_slam_viz.rviz')` 改为相对路径 `'src/my_sim_tesi_bringup/config/simple_slam_viz.rviz'`
- 修改文件：`/home/tridot/drone_ws_ori2/src/my_sim_tesi_bringup/launch/complete_rosbag_slam.launch.py` 第 127 行

**问题2（已解决）**：建立适用于 unitree g1 rosbag2 的地图当中，初始时 odom 的坐标位于地图水平面以下，且 odom 和 base_link 之间存在较大的 x,y 偏移。

**解决方案**：
- 问题原因：
  - SportModeState 中的位置数据是绝对位置，不是相对于起始位置的偏移
  - 对 odom 坐标系的理解误区：odom 应该固定在起始位置，base_link 随机器人移动
- 修改内容：
  - 添加初始位置记录功能
  - x,y 坐标：计算相对偏移（当前位置 - 起始位置），保持正确的运动信息
  - z 坐标：固定为 0.72m，避免显示在地下
- 修改文件：`/home/tridot/drone_ws_ori2/src/my_sim_tesi_ros2_nodes/scripts/odometry_converter_node.py`
- 效果：
  - 机器人启动时 odom→base_link 的 x,y 接近 (0,0)
  - 随着机器人移动，x,y 偏移会增大（这是正常的）
  - z 坐标保持在合理高度
- 重要说明：
  - odom→base_link 的 x,y 差异反映机器人的累积移动距离，这是正确的行为
  - 不能将 x,y 固定为 (0,0)，否则会导致 SLAM 地图重复
  - ROS TF 系统中，odom 坐标系应该保持固定，而不是跟随机器人移动

**问题3（已解决）**：base_link、livox_frame 和 livox_frame_corrected 的坐标方向不一致，在 RViz 中显示 base_link 和 livox_frame 的 x,y 轴与 livox_frame_corrected 有 180° 方向差。

**问题分析过程**：
- 初始误解：尝试在 TF 层面解决方向不一致问题
- 发现根本原因：系统中存在双层坐标修正机制
  1. **数据层修正**：`ground_segmentation_node.cc` 中对点云数据进行 y,z 轴反向 (`point.y = -point.y; point.z = -point.z;`)
  2. **TF层修正**：在 launch 文件中的静态变换发布器
- 问题实质：双重修正导致坐标系视觉不一致，虽然数据处理正确

**最终解决方案**：
- 修改策略：**统一所有坐标系方向，让数据层修正独立完成坐标转换**
- 修改内容：
  - base_link→livox_frame：保持恒等变换（0° 旋转）
  - livox_frame→livox_frame_corrected：保持恒等变换（0° 旋转）
  - 让 `ground_segmentation_node.cc` 中的数据修正独立处理倒挂激光雷达的坐标转换
- 修改文件：`/home/tridot/drone_ws_ori2/src/my_sim_tesi_bringup/launch/complete_rosbag_slam.launch.py`
- 具体修改：
  ```python
  # base_link → livox_frame (第88行)
  arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'livox_frame']
  
  # livox_frame → livox_frame_corrected (第17行)  
  arguments=['0', '0', '0', '0', '0', '0', 'livox_frame', 'livox_frame_corrected']
  ```
- 最终效果：
  - 所有坐标系（base_link、livox_frame、livox_frame_corrected）在 RViz 中显示相同的 x,y 轴方向
  - 点云数据通过 `ground_segmentation_node` 在数据层面完成倒挂激光雷达的修正
  - TF 系统保持所有坐标系方向一致，便于可视化和理解
  - 避免了双重修正带来的混淆
- 技术要点：
  - 数据修正和坐标系变换应该分离，避免双重修正
  - 倒挂 Livox MID-360 的坐标修正通过代码层面的数据处理完成
  - TF 系统保持坐标系方向的视觉一致性

## 已解决的所有问题

✅ **问题1**：RViz 无法显示地图（路径配置问题）  
✅ **问题2**：odom 和 base_link 之间的 TF 变换不正确（里程计转换问题）  
✅ **问题3**：坐标系方向不一致问题（双重修正导致的视觉混淆）

## 系统验证状态

**TF变换验证**：
- `base_link → livox_frame`: 0° 旋转 ✅
- `livox_frame → livox_frame_corrected`: 0° 旋转 ✅  
- `base_link → livox_frame_corrected`: 0° 旋转 ✅

**功能验证**：
- RViz 地图显示：正常 ✅
- 坐标系可视化：所有坐标系方向一致 ✅
- 点云数据处理：通过 ground_segmentation_node 正确修正 ✅
- SLAM 建图：功能正常 ✅

**关键设计理念**：
- **数据层修正**：在 `ground_segmentation_node.cc` 中处理倒挂激光雷达的坐标转换
- **TF层统一**：保持所有坐标系视觉方向一致，便于理解和调试
- **职责分离**：避免在多个地方重复应用相同的坐标修正


