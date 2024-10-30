from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='four_wheel_robot_description').find('four_wheel_robot_description')
    
    # 确保配置文件目录存在
    config_dir = os.path.join(pkg_share, 'config')
    if not os.path.exists(config_dir):
        os.makedirs(config_dir)
    rviz_config_dir = os.path.join(pkg_share, 'rviz')
    if not os.path.exists(rviz_config_dir):
        os.makedirs(rviz_config_dir)
    # RQT配置文件路径    

    rviz_config_path = os.path.join(rviz_config_dir, 'point_cloud.rviz')
    
    stereo_perspective_path = os.path.join(config_dir, 'stereo_view.perspective')
    lidar_perspective_path = os.path.join(config_dir, 'lidar_view.perspective')
    
    # 相机RQT节点
    camera_rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='camera_rqt_gui',
        arguments=['--perspective-file', stereo_perspective_path],
        parameters=[{'image_transport': 'compressed'}],  # 使用压缩图像传输
        output='screen'
    )
    
    # 激光雷达RQT节点
    lidar_rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='lidar_rqt_gui',
        arguments=['--perspective-file', lidar_perspective_path],
        output='screen'
    )
     # 点云可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    return LaunchDescription([
        camera_rqt_node,
        lidar_rqt_node,
        rviz_node
    ])