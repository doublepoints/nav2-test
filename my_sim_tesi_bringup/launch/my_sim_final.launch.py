import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node

SetEnvironmentVariable('USE_SIM_TIME', 'true'),

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('my_sim_tesi_bringup')
    pkg_project_gazebo = get_package_share_directory('my_sim_tesi_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # 获取 my_sim_tesi_ros2_nodes 包的路径，如果 odom_to_tf_node.py 在那里
    pkg_my_sim_tesi_ros2_nodes = get_package_share_directory('my_sim_tesi_ros2_nodes')

    sdf_file_path = os.path.join(pkg_project_gazebo, 'models', 'pioneer2dx', 'model.sdf')
    try:
        with open(sdf_file_path, 'r') as infp:
            robot_scan_description = infp.read()
    except FileNotFoundError:
        print(f"Error: The SDF file was not found at {sdf_file_path}")
        # Handle the error appropriately, maybe exit or use a default
        robot_scan_description = "" # Or raise an error

    robot_scan_urdf = """<?xml version="1.0"?>
    <robot name="scan_robot">
        <link name="base_link"/>
        <link name="lidar_link"/>
        <joint name="lidar_joint" type="fixed">
            <parent link="base_link"/>
            <child link="lidar_link"/>
            <origin xyz="0 0 0.1" rpy="0 0 0"/> </joint>
    </robot>
    """

    # Create a temporary URDF file for quadcopter
    quadcopter_urdf = """<?xml version="1.0"?>
    <robot name="quadcopter">
        <link name="base_link"/>
    </robot>
    """

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge_final_config.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'qos_overrides./quadcopter/rgbd_camera/image_raw.publisher.history': 'keep_last',
            'qos_overrides./quadcopter/rgbd_camera/image_raw.publisher.history_depth': '1',
            'qos_overrides./quadcopter/rgbd_camera/depth/image_raw.publisher.history': 'keep_last',
            'qos_overrides./quadcopter/rgbd_camera/depth/image_raw.publisher.history_depth': '1',
            'qos_overrides./quadcopter/rgbd_camera/points.publisher.history': 'keep_last',
            'qos_overrides./quadcopter/rgbd_camera/points.publisher.history_depth': '1',
            'qos_overrides./quadcopter/rgbd_camera/points.publisher.reliability': 'best_effort',
            'use_sim_time': True,
        }],
        output='screen'
    )

    robot_scan_static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_scan_static_transform_publisher_map',
        output='screen',
        parameters=[{'use_sim_time': True}],  # 确保添加这个参数
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'robot_scan/odom']
    )


    quadcopter_static_frame_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='quadcopter_static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'quadcopter/odom']
    )

    robot_scan_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='robot_scan',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_scan_urdf},
            #{'robot_description': robot_scan_description},
        ]
    )

    quadcopter_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='quadcopter',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': quadcopter_urdf},
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(pkg_project_bringup, 'config', 'final_simulation_config.rviz'),
            '--ros-args', '--log-level', 'WARN',
            '--param', 'tf_message_filter_queue_size:=100',
            '--param', 'tf_buffer_duration:=120.0'
        ],
        parameters=[{'use_sim_time': True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'my_world.sdf -r',
        ]),
        'use_sim_time' : 'true'
        }.items()
    )

    my_ros2_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'my_ros2_nodes.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_bringup, 'launch', 'nav2.launch.py')
        )
    )

    # 添加 TF 缓冲设置
    tf2_buffer_server = Node(
        package='tf2_ros',
        executable='buffer_server',
        name='tf2_buffer_server',
        parameters=[{
            'buffer_size': 120.0,
            'transform_tolerance': 1.0,  # 增加容错时间
            'transform_cache_time': 30.0  # 增加缓存时间
        }]
    )

    # 降低 TF 发布频率的静态转换发布器
    rgbd_camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rgbd_camera_static_tf',
        arguments=[
            # —— 平移：相机框架相对于 base_link 的 xyz（把它抬高到实际装配高度）
            '0', '0', '0.05',
            # —— 旋转：yaw, pitch, roll （都要按"Z Y X"顺序填）
            '0',   # yaw = 0° 
            '0',   # pitch = 0°，保持水平
            '0',   # roll = 0°
            'quadcopter/base_link',
            'quadcopter/base_link/rgbd_camera'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # 添加quadcopter到base_link的变换
    quadcopter_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='quadcopter_base_link_tf',
        arguments=[
            '0', '0', '0',  # 平移
            '0', '0', '0', # 旋转
            'quadcopter',
            'quadcopter/base_link'
        ],
        parameters=[{'use_sim_time': True}]
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/quadcopter/rgbd_camera/points'),
            ('scan', '/quadcopter/scan')
        ],
        parameters=[os.path.join(pkg_project_bringup, 'config', 'pointcloud_to_laserscan.yaml')]
    )

    # 添加全局TF参数
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '13'),
        SetEnvironmentVariable('TF_BUFFER_DURATION', '120.0'),
        SetEnvironmentVariable('TF_MESSAGE_FILTER_QUEUE_SIZE', '100'),
        bridge,
        gz_sim,
        robot_scan_static_frame_map,
        quadcopter_static_frame_map,
        robot_scan_state_publisher,
        quadcopter_state_publisher,
        tf2_buffer_server,
        rgbd_camera_static_tf,
        quadcopter_base_link_tf,
        pointcloud_to_laserscan_node,
        rviz,
        TimerAction(
            period=20.0,
            actions=[
                nav2_launch,
            ]
        ),
        TimerAction(
            period=20.0,
            actions=[
                my_ros2_nodes_launch,
            ]
        ),
    ])
