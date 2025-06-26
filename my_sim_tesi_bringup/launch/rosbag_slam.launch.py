import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Odometry converter node - 将SportModeState转换为标准Odometry
    odometry_converter = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='odometry_converter_node.py',
        name='odometry_converter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/lf/odommodestate',
            'output_topic': '/g1/odometry',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        output='screen'
    )
    
    # Static transform from map to odom
    static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )
    
    # Static transform from base_link to livox_frame_corrected
    static_frame_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'livox_frame_corrected']
    )
    
    # SLAM toolbox node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(pkg_project_bringup, 'config', 'rosbag_slam_config.yaml')
        ],
        output='screen',
    )
    
    # RViz for visualization (optional)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'map_scan_config.rviz')],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time from rosbag'
        ),
        odometry_converter,
        static_frame_map,
        static_frame_laser,
        slam_toolbox,
        rviz
    ])