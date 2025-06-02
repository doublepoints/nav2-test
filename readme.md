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

20250529進捗-g1nav：
pointcloud2laserscanを利用し、pointcloudからlaerscanに変換できました。
その他に、床の影響を排除するために、linefit-ground-segmentationを使って、床と障害物のpointcloudを区別し,障害物のpointcloudを利用し、laserscanに変換しました。
以下は実装プロセスをメモする
laserscanに変換
１：ros2 launch my_sim_tesi_bringup livox_to_laserscan.launch.py
rviz2で可視化
２：ros2 run rviz2 rviz2 -d src/my_sim_tesi_bringup/config/livox_visualization.rviz
Pointcloudセグメンテーションする
３：ros2 launch linefit_ground_segmentation_ros segmentation.launch.py


20250602進捗-g1nav:
DDSをcycloneddsに変更したら、nav2の自律移動がうまくできなくて、
現行の自律移動を実装したいでしがら、まずcycloneddsを変更しないことは重要です。




