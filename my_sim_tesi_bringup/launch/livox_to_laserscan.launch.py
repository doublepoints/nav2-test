# my_sim_tesi_bringup/launch/livox_to_laserscan.launch.py
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    
    # 针对倒挂Livox MID-360的坐标修正
    # 绕X轴旋转180度：x不变，y和z取负
    livox_frame_corrected_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_corrected_tf',
        arguments=[
            '0', '0', '0',           # 平移 (x, y, z) = (0, 0, 0)
            str(math.pi), '0', '0',  # 旋转 (roll, pitch, yaw) = (π, 0, 0) 绕X轴180度
            'livox_frame',           # 父坐标系
            'livox_frame_corrected'  # 子坐标系（修正后的）
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    # Livox雷达点云转激光扫描节点 - 使用配置文件
    livox_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/segmentation/obstacle'),
            ('scan', '/g1/laserscan')
        ],
        parameters=[os.path.join(pkg_project_bringup, 'config', 'livox_to_laserscan.yaml')]
    )

    return LaunchDescription([
        livox_frame_corrected_tf,
        livox_pointcloud_to_laserscan_node,
    ])