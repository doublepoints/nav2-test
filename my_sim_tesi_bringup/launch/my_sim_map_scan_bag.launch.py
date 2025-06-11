import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')

    # 声明rosbag路径参数
    bag_path = LaunchConfiguration('bag_path')
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Path to the rosbag file'
    )

    # 播放rosbag
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen'
    )

    # 设置时钟
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # 添加静态TF发布器
    static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
    )

    # 添加base_link到odom的静态TF发布器
    static_frame_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link']
    )

    # 添加base_link到laser的静态TF发布器
    static_frame_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0.1', '0', '0', '0', '1', 'base_link', 'laser']
    )

    # 创建自定义的slam_toolbox配置文件
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'scan_topic': '/laserscan',
                'odom_topic': '/lowstate',
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'max_laser_range': 20.0,
                'minimum_time_interval': 0.2,
                'transform_timeout': 0.2,
                'update_rate': 5.0,
                'resolution': 0.05,
                'max_queue_size': 10,
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_time_interval': 0.2,
                'transform_timeout': 0.2,
                'update_rate': 5.0,
                'resolution': 0.05,
                'max_queue_size': 10,
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'use_scan_topic': True,
                'use_odom_topic': True,
                'use_imu_topic': False,
                'use_imu_data': False,
                'use_odom_data': True,
                'use_scan_data': True,
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'use_scan_topic': True,
                'use_odom_topic': True,
                'use_imu_topic': False,
                'use_imu_data': False,
                'use_odom_data': True,
                'use_scan_data': True
            }
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'map_scan_config.rviz')],
    )

    return LaunchDescription([
        bag_path_arg,
        use_sim_time_arg,
        rosbag_play,
        static_frame_map,
        static_frame_base,
        static_frame_laser,
        slam_toolbox,
        rviz
    ]) 