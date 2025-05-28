# my_sim_tesi_bringup/launch/livox_to_laserscan.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    
    # Livox雷达点云转激光扫描节点
    livox_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/livox/lidar'),
            ('scan', '/g1/laserscan')
        ],
        parameters=[os.path.join(pkg_project_bringup, 'config', 'livox_to_laserscan.yaml')]
    )

    return LaunchDescription([
        livox_pointcloud_to_laserscan_node,
    ])