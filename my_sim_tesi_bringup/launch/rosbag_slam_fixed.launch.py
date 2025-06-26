import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    
    # Odometry converter node - 关闭use_sim_time
    odometry_converter = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='odometry_converter_node.py',
        name='odometry_converter',
        parameters=[{
            'use_sim_time': False,  # 关闭仿真时间
            'input_topic': '/lf/odommodestate',
            'output_topic': '/g1/odometry',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        output='screen'
    )
    
    # Static transforms
    static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        parameters=[{'use_sim_time': False}]
    )
    
    # Connect base_link to livox_frame
    static_frame_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', '1', 'base_link', 'livox_frame'],
        parameters=[{'use_sim_time': False}]
    )
    
    # SLAM toolbox node - 关闭use_sim_time，实时处理
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': False,  # 关闭仿真时间，使用系统时间
            'scan_topic': '/g1/laserscan',
            'map_frame': 'map',
            'odom_frame': 'odom', 
            'base_frame': 'base_link',
            'resolution': 0.05,
            'max_laser_range': 20.0,
            'minimum_time_interval': 0.01,
            'transform_timeout': 0.5,
            'tf_buffer_duration': 30.0,
            'enable_interactive_mode': True,
            'minimum_travel_distance': 0.01,
            'minimum_travel_heading': 0.01,
            'scan_buffer_size': 100,
            'scan_buffer_maximum_scan_distance': 20.0,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'scan_topic_queue_size': 100,
            'map_update_interval': 1.0,
            'transform_publish_period': 0.02,
            'mode': 'mapping',
            # 添加时间同步相关参数
            'tf_tolerance': 0.5,  # 增加TF容错时间
            'scan_queue_size': 100  # 增加扫描队列大小
        }],
        output='screen'
    )
    
    # 添加一个时钟发布器（如果需要）
    # clock_publisher = Node(
    #     package='rosbag2_play',
    #     executable='rosbag2_play',
    #     name='clock_publisher',
    #     arguments=['--clock', '100'],
    #     output='screen'
    # )
    
    return LaunchDescription([
        odometry_converter,
        static_frame_map,
        static_frame_laser,
        slam_toolbox
    ])