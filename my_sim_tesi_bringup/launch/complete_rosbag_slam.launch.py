import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    
    # 1. Livox frame correction TF
    livox_frame_corrected_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_corrected_tf',
        arguments=[
            '0', '0', '0',           
            '0', '0', '0',           # 恒等变换，因为数据已经在代码中被修正
            'livox_frame',           
            'livox_frame_corrected'  
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    # 2. Pointcloud to laserscan conversion
    livox_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='livox_pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/segmentation/obstacle'),
            ('scan', '/g1/laserscan')
        ],
        parameters=[
            {'use_sim_time': True},  # 重要：使用仿真时间
            os.path.join(pkg_project_bringup, 'config', 'livox_to_laserscan.yaml')
        ]
    )
    
    # 3. Ground segmentation
    ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            '/home/tridot/drone_ws_ori2/src/linefit_ground_segmentation_ros/launch/segmentation_params.yaml'
        ],
        remappings=[
            ('input_cloud', '/livox/lidar_3GGDJ6A00100021'),  # 输入点云
            ('output_ground', '/segmentation/ground'),
            ('output_obstacle', '/segmentation/obstacle')
        ]
    )
    
    # 4. Odometry converter
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
    
    # 5. Static transforms
    static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )
    
    static_frame_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'livox_frame'],
        parameters=[{'use_sim_time': True}]
    )
    
    # 6. SLAM toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': '/g1/laserscan',
            'map_frame': 'map',
            'odom_frame': 'odom', 
            'base_frame': 'base_link',
            'resolution': 0.05,
            'max_laser_range': 20.0,
            'minimum_time_interval': 0.1,
            'transform_timeout': 1.0,
            'tf_buffer_duration': 60.0,
            'enable_interactive_mode': True,
            'minimum_travel_distance': 0.05,
            'minimum_travel_heading': 0.05,
            'scan_buffer_size': 100,
            'scan_buffer_maximum_scan_distance': 20.0,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'scan_topic_queue_size': 100,
            'map_update_interval': 2.0,
            'transform_publish_period': 0.02
        }],
        output='screen'
    )
    
    # 7. RViz2 for visualization - 使用相对路径启动以正确显示地图
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'src/my_sim_tesi_bringup/config/simple_slam_viz.rviz'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        livox_frame_corrected_tf,
        ground_segmentation_node,
        livox_pointcloud_to_laserscan_node,
        odometry_converter,
        static_frame_map,
        static_frame_laser,
        slam_toolbox,
        rviz2_node
    ])