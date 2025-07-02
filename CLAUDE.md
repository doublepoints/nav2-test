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

# （Option）启动CycloneDDS
source /home/tridot/CProjects/unitree_ros2/setup_default.sh
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

# 终端 3：保存地图
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


## 功能追加修改

### 2025-07-02 新增功能：实机导航系统

**需求**：创建可直接部署到 G1 机器人的导航系统，结合仿真导航功能和实机建图的话题配置。

**实现方案**：
创建了三个新文件，构建完整的实机导航系统：

#### 1. 主启动文件：`robot_navigation.launch.py`
整合了以下功能模块：
- **传感器处理链**：
  - 地面分割：`linefit_ground_segmentation_ros` 处理 Livox 点云数据
  - 点云转激光：将分割后的障碍物点云转换为 2D 激光扫描
  - 里程计转换：将 `/lf/odommodestate` 转换为标准 ROS2 里程计消息
  
- **TF 坐标系**：
  ```
  map → odom → base_link → livox_frame → livox_frame_corrected
  ```
  
- **导航栈组件**：
  - AMCL：自适应蒙特卡洛定位
  - Nav2 规划器：全局路径规划
  - Nav2 控制器：局部路径跟踪
  - 行为服务器：恢复行为（旋转、后退等）
  - 速度平滑器：平滑速度指令输出

#### 2. 导航参数配置：`robot_nav2_params.yaml`
- **实机配置**：所有节点设置 `use_sim_time: False`
- **话题映射**：
  - 激光扫描：`/g1/laserscan`
  - 里程计：`/g1/odometry`
  - 速度控制：`/g1/cmd_vel`
- **机器人尺寸**：根据 G1 实际尺寸调整 footprint 为 `[[0.4,0.3],[-0.4,0.3],[-0.4,-0.3],[0.4,-0.3]]`
- **传感器参数**：激光最大距离设为 20 米（适配 Livox MID-360）

#### 3. 可视化配置：`robot_navigation.rviz`
配置了导航所需的所有可视化组件：
- 地图显示
- 局部/全局代价地图
- 激光扫描数据
- 路径规划结果
- TF 坐标系
- 导航目标设置工具

### 使用方法

**实机导航（需要 CycloneDDS）：**
```bash
# 环境设置（所有终端都需要执行）
source /home/tridot/CProjects/unitree_ros2/setup_default.sh
source install/setup.bash

# 终端 1：启动导航系统
ros2 launch my_sim_tesi_bringup robot_navigation.launch.py

# 终端 2：在 RViz 中设置导航目标
# 使用 "2D Goal Pose" 工具点击目标位置

# 注意：需要先有地图文件，请确保 map.yaml 路径正确
```

### 技术要点

1. **保持原有架构**：所有参数名、变量名与仿真版本保持一致，便于维护
2. **实机适配**：
   - 关闭仿真时间（use_sim_time: False）
   - 使用实际传感器话题
   - 调整机器人物理参数
3. **模块化设计**：传感器处理、定位、导航各模块独立，便于调试
4. **延迟启动**：导航栈延迟 5 秒启动，确保 TF 树建立完成

### 相关文件路径
- 启动文件：`src/my_sim_tesi_bringup/launch/robot_navigation.launch.py`
- 参数配置：`src/my_sim_tesi_bringup/config/robot_nav2_params.yaml`
- 可视化配置：`src/my_sim_tesi_bringup/config/robot_navigation.rviz`
- 地图文件：配置在 `robot_nav2_params.yaml` 中，当前指向 `my_map1.yaml`

### 使用 Rosbag 测试导航功能

在部署到实体机器人之前，可以使用录制的 rosbag 文件测试导航功能的正确性。

#### 测试文件
1. **`rosbag_navigation_test.launch.py`** - Rosbag 导航测试启动文件
   - 所有节点配置 `use_sim_time: True` 以使用 rosbag 时间戳
   - 导航命令输出到 `/g1/cmd_vel_nav` 测试话题，避免影响原始数据
   - 延迟 10 秒启动导航栈，确保传感器数据准备就绪

2. **`rosbag_nav2_test_params.yaml`** - Rosbag 测试专用参数配置
   - 所有 `use_sim_time` 设置为 True
   - 地图文件路径：`/home/tridot/drone_ws_ori2/src/my_sim_tesi_bringup/map/my_map1.yaml`
   - 其他参数与实机配置保持一致

3. **`rosbag_navigation_test.rviz`** - Rosbag 测试可视化配置
   - 包含地图、代价地图、激光扫描、路径等所有导航相关显示

#### 测试步骤

```bash
# 环境设置（所有终端都需要执行）
source /home/tridot/CProjects/unitree_ros2/setup_default.sh
source install/setup.bash

# 终端 1：启动导航测试系统
ros2 launch my_sim_tesi_bringup rosbag_navigation_test.launch.py

# 终端 2：播放 rosbag（使用 --clock 发布时钟信息）
ros2 bag play /path/to/your/rosbag --clock
# 例如：ros2 bag play /media/tridot/DATA2/dataset2/G1/rosbag2_2025_05_26-15_31_21 --clock

# 终端 3：监控导航输出
ros2 topic echo /g1/cmd_vel_nav

# 在 RViz 中进行导航测试：
# 1. 等待地图加载完成（应该能看到灰色地图）
# 2. 使用 "2D Pose Estimate" 设置机器人初始位置
# 3. 使用 "2D Goal Pose" 设置导航目标
# 4. 观察路径规划和代价地图更新
```

#### 验证要点

1. **地图显示**：RViz 中应正确显示地图（灰色背景）
2. **TF 树完整性**：检查 `map → odom → base_link → livox_frame` 变换链
3. **激光数据转换**：确认 `/g1/laserscan` 话题有数据输出
4. **定位准确性**：AMCL 粒子云应收敛到正确位置
5. **路径规划**：设置目标后应生成绿色路径
6. **避障功能**：局部代价地图应正确显示障碍物
7. **速度指令**：`/g1/cmd_vel_nav` 应输出合理的速度命令

#### 关键检查命令

```bash
# 检查地图话题
ros2 topic echo /map --once

# 检查激光扫描数据
ros2 topic echo /g1/laserscan --once

# 检查里程计数据
ros2 topic echo /g1/odometry --once

# 检查 TF 树
ros2 run tf2_tools view_frames

# 查看所有导航相关话题
ros2 topic list | grep -E "(map|nav|cost|plan|goal)"
```

#### 常见问题及解决

1. **地图不显示**：
   - 检查 `rosbag_nav2_test_params.yaml` 中的地图路径
   - 确认地图文件 `my_map1.yaml` 和 `my_map1.pgm` 存在

2. **TF 变换超时**：
   - 增加 `transform_tolerance` 参数
   - 检查 rosbag 是否包含必要的里程计数据

3. **激光数据缺失**：
   - 确认 rosbag 包含 `/livox/lidar_3GGDJ6A00100021` 点云话题
   - 检查地面分割节点是否正常运行

4. **定位失败**：
   - 手动使用 "2D Pose Estimate" 设置初始位姿
   - 调整 AMCL 参数或增加粒子数量

### 实体机器人导航部署

#### 部署准备

1. **确保地图文件可用**：
   - 地图文件：`src/my_sim_tesi_bringup/map/my_map1.yaml`
   - 确认地图覆盖导航区域

2. **环境检查**：
   - G1 机器人硬件正常
   - Livox MID-360 激光雷达工作正常
   - CycloneDDS 环境配置正确

#### 实机导航启动

```bash
# 环境设置（所有终端都需要执行）
source /home/tridot/CProjects/unitree_ros2/setup_default.sh
source install/setup.bash
export ROS_DOMAIN_ID=13

# 终端 1：启动实机导航系统
ros2 launch my_sim_tesi_bringup robot_navigation.launch.py

# 终端 2：监控导航状态
ros2 topic echo /g1/cmd_vel

# 在 RViz 中：
# 1. 等待系统初始化完成
# 2. 使用 "2D Pose Estimate" 设置机器人在地图中的初始位置
# 3. 使用 "2D Goal Pose" 设置导航目标
# 4. 机器人将自动导航到目标位置
```

#### 实机导航配置文件

- **启动文件**：`robot_navigation.launch.py`
- **参数配置**：`robot_nav2_params.yaml`（`use_sim_time: False`）
- **可视化配置**：`robot_navigation.rviz`
- **地图文件**：`my_map1.yaml`

#### 系统验证

通过 rosbag 测试验证以下功能正常后再部署实机：
- 传感器数据处理链的正确性
- 导航算法参数的合理性  
- 路径规划和避障的有效性
- 系统整体集成的稳定性

**重要**：rosbag 测试成功后，切换到 `robot_navigation.launch.py` 即可在实体机器人上运行相同的导航功能。