import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    
    # Odometry converter node
    odometry_converter = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='odometry_converter_node.py',
        name='odometry_converter',
        parameters=[{
            'use_sim_time': True,
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
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )
    
    # Connect base_link to livox_frame (保持原有的livox_frame→livox_frame_corrected变换)
    static_frame_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', '1', 'base_link', 'livox_frame']
    )
    
    # SLAM toolbox node - minimal configuration
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/g1/laserscan',
            'odom_topic': '/g1/odometry',
            'map_frame': 'map',
            'odom_frame': 'odom', 
            'base_frame': 'base_link',
            'resolution': 0.05,
            'max_laser_range': 15.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'enable_interactive_mode': True
        }],
        output='screen'
    )
    
    return LaunchDescription([
        odometry_converter,
        static_frame_map,
        static_frame_laser,
        slam_toolbox
    ])