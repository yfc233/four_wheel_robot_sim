#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('four_wheel_robot_navigation').find('four_wheel_robot_navigation')
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    
    # 配置文件路径
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start nav2'
    )
    
    # Nav2 节点
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_config,
            'map': ''  # 使用SLAM时不需要地图
        }.items()
    )
    
    # SLAM 节点
    slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_laser_range': 20.0,
            'resolution': 0.05,
            'map_update_interval': 5.0,
            'max_update_rate': 1.0,
            'enable_interactive_mode': True,
            'transform_timeout': 0.2,
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'odom_frame': 'odom'
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        slam_toolbox_cmd,
        nav2_bringup_cmd
    ]) 