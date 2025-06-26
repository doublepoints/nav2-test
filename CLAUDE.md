# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

### Environment Setup (Required before any commands)
```bash
# Source the workspace
source install/setup.bash

# Set Gazebo resource paths (adjust paths as needed)
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share

# Set library paths
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH

# Set ROS domain (to avoid conflicts)
export ROS_DOMAIN_ID=13

# Deactivate anaconda if using conda environments
```

### Build Commands
```bash
# Build the entire workspace (run from workspace root)
colcon build

# Build specific package
colcon build --packages-select <package_name>
```

### Primary Launch Commands

**Navigation and Localization:**
```bash
ros2 launch my_sim_tesi_bringup my_sim_final.launch.py
```

**SLAM Mapping:**
```bash
ros2 launch my_sim_tesi_bringup my_sim_map_scan.launch.py
# In Gazebo GUI, enable "Key Publisher" plugin for robot control
```

**Save Map (after SLAM):**
```bash
ros2 run nav2_map_server map_saver_cli
```

**PointCloud Processing (requires CycloneDDS):**
```bash
# Terminal 1: Start pointcloud to laserscan conversion
ros2 launch my_sim_tesi_bringup livox_to_laserscan.launch.py

# Terminal 2: Start ground segmentation
ros2 launch linefit_ground_segmentation_ros segmentation.launch.py

# Terminal 3: Play rosbag (if using recorded data)
ros2 bag play /path/to/bag --loop

# Terminal 4: Visualization
rviz2 -d src/my_sim_tesi_bringup/config/livox_visualization.rviz
```

## Architecture Overview

This is a ROS2 workspace for drone/robot simulation, mapping, and navigation with the following key components:

### Core Packages

**my_sim_tesi_bringup**: Main launch files and configuration
- Contains launch files for different simulation modes
- Configuration files for SLAM, navigation, and sensor processing
- RViz configurations for visualization

**my_sim_tesi_gazebo**: Gazebo simulation environment  
- Custom world files and robot models (Pioneer2DX, quadcopter)
- 3D models and textures for simulation environments
- SDF model definitions

**my_sim_tesi_ros2_nodes**: Custom ROS2 nodes
- Action servers/clients for pose control
- Orchestrator node for system coordination  
- PointCloud to LaserScan conversion nodes
- Odometry processing utilities

**my_sim_tesi_ros2_interfaces**: Custom message/action definitions
- DronePoseControl action interface

**linefit_ground_segmentation**: Ground plane segmentation
- Separates ground points from obstacles in point clouds
- Used for navigation in complex 3D environments

### System Data Flow

1. **Simulation**: Gazebo provides sensor data (lidar, odometry, camera)
2. **Bridge**: ros_gz_bridge transfers data between Gazebo and ROS2
3. **Processing**: Custom nodes process sensor data (ground segmentation, coordinate transforms)
4. **SLAM/Navigation**: slam_toolbox or nav2 for mapping and navigation
5. **Visualization**: RViz2 displays maps, robot state, and sensor data

### Key Configuration Files

- `ros_gz_bridge_*_config.yaml`: Defines topic bridges between Gazebo and ROS2
- `slam_toolbox_config.yaml`: SLAM algorithm parameters
- `nav2_param_config.yaml`: Navigation stack configuration
- `*.rviz`: Visualization configurations for different use cases

### TF Frame Structure
```
map → robot_scan/odom → robot_scan/base_footprint → [sensor frames]
```

### Important Topics
- `/robot_scan/scan`: Laser scan data for SLAM/navigation
- `/robot_scan/odometry`: Robot odometry 
- `/map`: Occupancy grid map output from SLAM
- `/tf`: Dynamic coordinate transforms
- `/robot_scan/cmd_vel`: Velocity commands for robot control

## Development Notes

- This workspace supports both ground robot (Pioneer2DX) and drone (quadcopter) simulation
- The system can process both 2D laser scans and 3D point clouds
- Ground segmentation is particularly important for 3D point cloud processing
- Multiple simulation scenarios are supported through different launch files
- Uses Ignition Gazebo (not classic Gazebo)