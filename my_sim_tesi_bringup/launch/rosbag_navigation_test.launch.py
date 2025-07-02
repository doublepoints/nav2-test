import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        parameters=[{'use_sim_time': True}]  # 使用 rosbag 时间
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
            {'use_sim_time': True},  # 使用 rosbag 时间
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
            {'use_sim_time': True},  # 使用 rosbag 时间
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
            'use_sim_time': True,  # 使用 rosbag 时间
            'input_topic': '/lf/odommodestate',
            'output_topic': '/g1/odometry',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        output='screen'
    )
    
    # 5. Static transforms
    static_frame_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )
    
    static_frame_base_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'livox_frame'],
        parameters=[{'use_sim_time': True}]
    )
    
    # 6. Robot state publisher for base_link
    robot_urdf = """<?xml version="1.0"?>
    <robot name="g1_robot">
        <link name="base_link"/>
    </robot>
    """
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},  # 使用 rosbag 时间
            {'robot_description': robot_urdf},
        ]
    )
    
    # 7. TF2 buffer server for better TF performance
    tf2_buffer_server = Node(
        package='tf2_ros',
        executable='buffer_server',
        name='tf2_buffer_server',
        parameters=[{
            'use_sim_time': True,  # 使用 rosbag 时间
            'buffer_size': 120.0,
            'transform_tolerance': 1.0,
            'transform_cache_time': 30.0
        }]
    )
    
    # 8. Navigation configuration for rosbag testing
    nav2_config_file = os.path.join(pkg_project_bringup, 'config', 'rosbag_nav2_test_params.yaml')
    
    # 9. Nav2 lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output="screen",
        parameters=[
            {"use_sim_time": True},  # 使用 rosbag 时间
            {"autostart": True},
            {"node_names": ["map_server",
                            "controller_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "amcl",
                            "velocity_smoother"]}
        ]
    )
    
    # 10. AMCL for localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_config_file],
    )
    
    # 11. Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_config_file],
    )
    
    # 12. Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config_file],
    )
    
    # 13. Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config_file],
    )
    
    # 14. Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config_file],
    )
    
    # 15. BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config_file],
    )
    
    # 16. Velocity smoother - 输出到一个测试话题，避免影响 rosbag 数据
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config_file],
        remappings=[('/cmd_vel_smoothed', '/g1/cmd_vel_nav')]  # 输出到测试话题
    )
    
    # 17. RViz2 for visualization
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(pkg_project_bringup, 'config', 'rosbag_navigation_test.rviz'),
            '--ros-args', '--log-level', 'WARN',
            '--param', 'tf_message_filter_queue_size:=100',
            '--param', 'tf_buffer_duration:=120.0'
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    # Launch sequence
    return LaunchDescription([
        # Environment variables
        SetEnvironmentVariable('ROS_DOMAIN_ID', '13'),
        SetEnvironmentVariable('TF_BUFFER_DURATION', '120.0'),
        SetEnvironmentVariable('TF_MESSAGE_FILTER_QUEUE_SIZE', '100'),
        
        # Core TF and sensor processing nodes
        livox_frame_corrected_tf,
        static_frame_map_odom,
        static_frame_base_livox,
        robot_state_publisher,
        tf2_buffer_server,
        
        # Sensor processing
        ground_segmentation_node,
        livox_pointcloud_to_laserscan_node,
        odometry_converter,
        
        # Navigation stack with delay to ensure TF is ready
        TimerAction(
            period=10.0,  # 给更多时间让传感器数据准备好
            actions=[
                lifecycle_manager,
                amcl,
                map_server,
                planner_server,
                behavior_server,
                controller_server,
                bt_navigator,
                velocity_smoother,
            ]
        ),
        
        # Visualization
        rviz2_node,
    ])