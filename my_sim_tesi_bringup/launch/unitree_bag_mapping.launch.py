import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')

    # 声明rosbag路径参数
    bag_path = LaunchConfiguration('bag_path')
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the unitree rosbag file'
    )

    # 设置时钟
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # 播放rosbag
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--rate', '1.0'],
        output='screen'
    )

    # 静态TF发布器 - map到odom
    static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 静态TF发布器 - base_link到laser_link
    # 根据实际的激光雷达安装位置调整这些值
    static_frame_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', '1', 'base_link', 'laser_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Unitree SportModeState到标准Odometry的转换节点
    odometry_converter = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='odometry_converter_node.py',
        name='odometry_converter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_topic': '/lf/odommodestate'},
            {'output_topic': '/odom'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'publish_tf': False}  # 我们使用单独的TF发布器
        ]
    )

    # odom到base_link的TF发布器
    odom_to_tf_node = Node(
        package='my_sim_tesi_ros2_nodes',
        executable='odom_to_tf_node.py',
        name='odom_to_tf_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'}
        ],
        remappings=[('/odom', '/odom')]
    )

    # SLAM Toolbox配置
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'scan_topic': '/g1/laserscan',
                'odom_topic': '/odom',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'max_laser_range': 20.0,
                'minimum_time_interval': 0.2,
                'transform_timeout': 0.5,
                'transform_publish_period': 0.02,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'max_queue_size': 10,
                'minimum_travel_distance': 0.3,
                'minimum_travel_heading': 0.3,
                'tf_buffer_duration': 30.0,
                'stack_size_to_use': 40000000,
                'enable_interactive_mode': True,
                
                # 扫描匹配参数
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_time_interval': 0.5,
                'transform_timeout': 0.2,
                
                # 处理器参数
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 10.0,
                'link_match_minimum_response_fine': 0.1,
                'link_scan_maximum_distance': 1.5,
                'loop_search_maximum_distance': 3.0,
                'do_loop_closing': True,
                'loop_match_minimum_chain_size': 10,
                'loop_match_maximum_variance_coarse': 3.0,
                'loop_match_minimum_response_coarse': 0.35,
                'loop_match_minimum_response_fine': 0.45,
                
                # 相关器参数
                'correlation_search_space_dimension': 0.5,
                'correlation_search_space_resolution': 0.01,
                'correlation_search_space_smear_deviation': 0.1,
                
                # 循环闭合参数
                'loop_search_space_dimension': 8.0,
                'loop_search_space_resolution': 0.05,
                'loop_search_space_smear_deviation': 0.03,
                
                # 扫描匹配参数
                'distance_variance_penalty': 0.5,
                'angle_variance_penalty': 1.0,
                'fine_search_angle_offset': 0.00349,
                'coarse_search_angle_offset': 0.349,
                'coarse_angle_resolution': 0.0349,
                'minimum_angle_penalty': 0.9,
                'minimum_distance_penalty': 0.5,
                'use_response_expansion': True
            }
        ],
        output='screen',
    )

    # RViz配置
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'map_scan_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        bag_path_arg,
        use_sim_time_arg,
        rosbag_play,
        static_frame_map,
        static_frame_laser,
        # 添加延迟启动，确保rosbag先开始播放
        TimerAction(
            period=3.0,
            actions=[
                odometry_converter,
                #odom_to_tf_node,
            ]
        ),
        TimerAction(
            period=6.0,
            actions=[
                slam_toolbox,
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                rviz,
            ]
        ),
    ])