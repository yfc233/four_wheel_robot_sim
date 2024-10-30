#!/usr/bin/env python3
"""
四轮机器人Gazebo仿真启动文件

此launch文件用于启动完整的仿真环境，包括：
- Gazebo仿真器
- 机器人状态发布
- 坐标变换
- RViz2可视化
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """
    生成启动描述
    
    Returns:
        LaunchDescription: 包含所有节点和配置的启动描述对象
    """
    # 获取包路径
    pkg_gazebo = FindPackageShare('four_wheel_robot_gazebo').find('four_wheel_robot_gazebo')
    pkg_description = FindPackageShare('four_wheel_robot_description').find('four_wheel_robot_description')
    
    # 设置Gazebo世界文件路径
    world_file = os.path.join(pkg_gazebo, 'worlds', 'test_world.world')
    
    # 获取URDF文件路径
    urdf_model_path = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    
    # 启动Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros').find('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'gui': 'true'
        }.items()
    )

    # 静态坐标变换
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # 基座到里程计坐标变换
    base_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # 里程计到基座坐标变换
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )
    
    # 机器人状态发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_model_path]),
            'use_sim_time': True,
            'publish_frequency': 50.0,
            'frame_prefix': '',
        }]
    )
    
    # 在Gazebo中生成机器人
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'four_wheel_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
  
    # 添加RViz配置
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'urdf_config.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 创建并返回启动描述
    return LaunchDescription([
        # 1. 首先启动Gazebo
        gazebo,
        
        # 2. 发布机器人描述
        robot_state_publisher,
        
        # 3. 发布关节状态
        joint_state_publisher,
        
        # 4. 发布TF树
        static_tf,
        map_to_odom,
        odom_to_base,
        base_to_footprint,
        
        # 5. 生成机器人
        spawn_robot,
        
        # 6. 启动RViz
        rviz2
    ])

    
