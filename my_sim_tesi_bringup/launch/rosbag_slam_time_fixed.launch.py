import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    
    # 使用时间同步的odometry转换器
    odometry_converter = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='odometry_converter_time_sync.py',
        name='odometry_converter',
        parameters=[{
            'use_sim_time': True,  # 启用仿真时间
            'input_topic': '/lf/odommodestate',
            'output_topic': '/g1/odometry',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
            'use_message_time': False  # 使用相对时间而不是消息时间戳
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
        parameters=[{'use_sim_time': True}]
    )
    
    # Connect base_link to livox_frame
    static_frame_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', '1', 'base_link', 'livox_frame'],
        parameters=[{'use_sim_time': True}]
    )
    
    # SLAM toolbox node with proper time handling
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': True,  # 使用仿真时间
            'scan_topic': '/g1/laserscan',
            'map_frame': 'map',
            'odom_frame': 'odom', 
            'base_frame': 'base_link',
            'resolution': 0.05,
            'max_laser_range': 20.0,
            'minimum_time_interval': 0.01,
            'transform_timeout': 2.0,  # 增加超时时间
            'tf_buffer_duration': 60.0,
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
            'mode': 'mapping'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        odometry_converter,
        static_frame_map,
        static_frame_laser,
        slam_toolbox
    ])