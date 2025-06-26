#为了执行，需要在编译后添加下面两个指令：
IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws/install/my_sim_tesi_gazebo/share

export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH

#为了避免和其他程序起冲突，从而使用别的空间
export ROS_DOMAIN_ID=13


#为了执行，需要在编译后添加下面两个指令：
IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share

export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH

#为了避免和其他程序起冲突，从而使用别的空间
export ROS_DOMAIN_ID=13


启动顺序介绍：

统一项：
注意：如果使用anaconda,需要将anaconda的虚拟环境先无效化，否则会出现冲突
#1：项目加载环境设置
source install/setup.bash
#2:gazebo，ros2环境设置
#下面的需要修改文件路径，需要注意
IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/tridot/drone_ws_ori2/src/my_sim_tesi_gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}:/home/tridot/drone_ws_ori2/install/my_sim_tesi_gazebo/share

export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export IGN_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu:$IGN_PLUGIN_PATH

#为了避免和其他程序起冲突，从而使用别的空间
export ROS_DOMAIN_ID=13

#3：（可选）如果需要夹杂cyclonedds,需要按照实际环境来设置
source /home/tridot/CProjects/unitree_ros2/setup_default.sh


功能一：定位，导航

ros2 launch my_sim_tesi_bringup my_sim_final.launch.py

功能二：将G1的pointcloud 进行地面和障碍物区分,并转化为laserscan,需要cyclonedds

第一个终端，启动pointcloud2laserscan node

ros2 launch my_sim_tesi_bringup livox_to_laserscan.launch.py

第二个终端，启动pointclou的地面和障碍物分割功能

ros2 launch linefit_ground_segmentation_ros segmentation.launch.py

第三个终端，播放rosbag（如果有实际数据流，可以忽略）

ros2 bag play /media/tridot/DATA2/dataset2/G1/rosbag2_2025_05_26-15_31_21/ --loop

第四个终端，启动rviz2,可视化
rviz2 -d src/my_sim_tesi_bringup/config/livox_visualization.rviz 

功能三：建图(适用于gazebo)

1：启动建图
ros2 launch my_sim_tesi_bringup my_sim_map_scan.launch.py

（在启动的gazebo2中，输入key publihser来启动，在gazebo2中对无人小车的控制功能）

2：保存地图（需要新的终端）
ros2 run nav2_map_server map_saver_cli


功能四：建图(适用于g1 rosbag)

1：启动建图
ros2 launch my_sim_tesi_bringup complete_rosbag_slam.launch.py

2：显示地图

方法1：./src/my_sim_tesi_bringup/scripts/diagnose_rviz.sh
方法2：rviz2 -d src/my_sim_tesi_bringup/config/simple_slam_viz.rviz


3：保存地图（需要新的终端）--未测试
ros2 run nav2_map_server map_saver_cli



