from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('four_wheel_robot_navigation').find('four_wheel_robot_navigation')
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    
    # 设置参数文件路径
    nav_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # 包含Nav2 Bringup launch文件
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav_params_path,
            'autostart': autostart
        }.items()
    )
    
    # RVIZ配置
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2_config.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start up the nav2 stack'),
            
        nav2_bringup_launch,
        rviz_node
    ]) 