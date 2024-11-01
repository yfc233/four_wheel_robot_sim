from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown as ShutdownEvent
import os
from launch.actions import TimerAction

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('four_wheel_robot_nav').find('four_wheel_robot_nav')
    description_pkg_share = FindPackageShare('four_wheel_robot_description').find('four_wheel_robot_description')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    
    # 定义路径
    bt_xml_path = PathJoinSubstitution([pkg_share, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'])
    nav2_params_path = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'nav.rviz'])
    world_file_path = PathJoinSubstitution([pkg_share, 'worlds', 'empty.world'])
    print(f"Nav package path: {pkg_share}")
    print(f"Description package path: {description_pkg_share}")
    print(f"Nav2 params path: {nav2_params_path}")
    print(f"Rviz config path: {rviz_config_path}")
    print(f"World file path: {world_file_path}")
    
    # 设置参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Gazebo 启动配置
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'verbose': 'false',
            'world': world_file_path
        }.items()
    )

    # 修改 joint_state_publisher 配置
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 50,  # 提高发布频率
            'publish_default_positions': True,
            'source_list': ['joint_states']
        }],
        output='screen'
    )
    
    # 修改 robot_state_pub 配置
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
            'frame_prefix': '',  # 确保没有额外的frame前缀
            'robot_description': Command([
                FindExecutable(name='xacro'), ' ',
                os.path.join(description_pkg_share, 'urdf', 'robot.urdf.xacro')
            ])
        }]
    )
    
    # 修改 spawn_entity 配置
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'four_wheel_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-unpause'
        ]
    )

    # 添加SLAM参数
    slam_params = {
        'use_sim_time': use_sim_time,
        'base_frame': 'base_link',
        'odom_frame': 'odom',
        'map_frame': 'map',
        
        # 激光雷达配置
        'scan_topic': '/scan',
        'max_laser_range': 20.0,
        
        # SLAM参数化
        'resolution': 0.05,
        'transform_publish_period': 0.02,  # 提高TF发布频率
        'map_update_interval': 1.0,        # 提高地图更新频率
        'minimum_time_interval': 0.1,      # 降低最时间间隔
        'transform_timeout': 0.1,          # 降低超时时间
        'tf_buffer_duration': 10.0,        # 降低缓存时间
        
        # SLAM性能参数
        'stack_size_to_use': 40000000,
        'enable_interactive_mode': False,
        
        # 回环检测参数
        'loop_search_maximum_distance': 3.0,
        'loop_match_minimum_chain_size': 3,
        'loop_match_maximum_variance_coarse': 3.0,
        'loop_match_minimum_response_coarse': 0.35,
        'loop_match_minimum_response_fine': 0.45,
        
        # 确保发布TF变换
        'publish_frame_transforms': True
    }

    # 添加环境变量设置
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # 修改 slam_toolbox_node 配置
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'max_laser_range': 20.0,
            'minimum_time_interval': 0.2,
            'transform_publish_period': 0.02,
            'map_update_interval': 5.0,
            'resolution': 0.05,
            'max_update_rate': 10.0,
            'publish_period': 0.1,
            'enable_interactive_mode': False,
            'publish_frame_transforms': True,
            'map_to_odom_transform': True  # 确保SLAM发布map到odom的变换
        }]
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom')
        ]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_path,
            {'default_bt_xml_filename': bt_xml_path}
        ]
    )

    recoveries_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[nav2_params_path]
    )

    # 修改 lifecycle_manager 配置
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'slam_toolbox',
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'local_costmap',
                'global_costmap'
            ],
            'bond_timeout': 4.0,
            'attempt_respawn_reconnection': True
        }]
    )
   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 添加TF树监控节点
    tf_monitor_node = Node(
        package='tf2_ros',
        executable='tf2_monitor',
        name='tf2_monitor',
        output='screen'
    )

    # 定义协方差矩阵
    PROCESS_NOISE_COVAR = [
        0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.02, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.015
    ]

    # 添加话题重映射
    remappings = [
        ('/scan', '/four_wheel_robot/scan'),
        ('/cmd_vel', '/four_wheel_robot/cmd_vel'),
    ]
    
    # 修改SLAM节点配置
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
        remappings=remappings
    )
    
    # 修改导航节点配置
    nav_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_path],
            remappings=remappings
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_path],
            remappings=remappings
        ),
        # ... 其他导航节点 ...
    ]
    
    return LaunchDescription([
        stdout_linebuf_envvar,
        
        # 参数声明
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # 1. 启动基础节点
        robot_state_pub,
        joint_state_publisher,
        
        # 2. 启动Gazebo和机器人
        gazebo,
        TimerAction(
            period=2.0,
            actions=[spawn_entity]
        ),
        
        # 3. 启动IMU节点（确保odom到base_link的变换）
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'frequency': 30.0,
                        'two_d_mode': True,
                        'publish_tf': True,
                        'odom_frame': 'odom',
                        'base_link_frame': 'base_link',
                        'world_frame': 'odom',
                        'imu0': '/imu',
                        'imu0_config': [
                            False, False, False,  # x, y, z 位置
                            True,  True,  True,   # roll, pitch, yaw 角度
                            False, False, False,  # x, y, z 速度
                            True,  
                        ],
                        'imu0_differential': False,
                        'imu0_relative': False,
                        'print_diagnostics': True,
                        'process_noise_covariance': PROCESS_NOISE_COVAR
                    }]
                )
            ]
        ),
        
        # 4. 启动SLAM（处理map到odom的变换）
        TimerAction(
            period=4.0,
            actions=[slam_toolbox_node]
        ),
        
        # 5. 启动生命周期管理器
        TimerAction(
            period=5.0,
            actions=[lifecycle_manager]
        ),
        
        # 6. 启动导航相关节点
        TimerAction(
            period=6.0,
            actions=[
                controller_node,
                planner_server,
                recoveries_node,
                bt_navigator_node,
                
                
            ]
        ),
        
        # 7. 启动可视化和监控工具
        TimerAction(
            period=7.0,
            actions=[rviz_node]
        ),
        
        # 8. 保留两个TF监控节点（一个用于实时监控，一个用于定期报告）
        tf_monitor_node,  # 实时监控
        TimerAction(
            period=10.0,
            actions=[tf_monitor_node]  # 定期报告
        )
    ])