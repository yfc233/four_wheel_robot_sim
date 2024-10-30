# Four Wheel Robot Simulation

基于ROS2 Foxy的四轮机器人仿真项目。

## 功能特点
- 四轮差速驱动机器人模型
- 3D激光雷达和双目相机传感器
- Gazebo仿真环境
- RViz2可视化界面

## 依赖项
- ROS2 Foxy
- Gazebo 11
- rviz2
- xacro
- robot_state_publisher
- joint_state_publisher
- gazebo_ros_pkgs

## 安装步骤
1. 创建工作空间： 



bash
mkdir -p ~/four_wheel_robot_sim/src
cd ~/four_wheel_robot_sim/src

2. 克隆仓库：

git clone https://github.com/yourusername/four_wheel_robot_sim.git

3. 安装依赖：

cd ~/four_wheel_robot_sim   
colcon build

4. 配置环境变量：

source install/setup.bash       

5. 启动仿真环境：

ros2 launch four_wheel_robot_gazebo gazebo.launch.py    

6. 启动RViz2可视化：

ros2 launch four_wheel_robot_rviz rviz_config.launch.py

## 注意事项
- 请确保所有依赖项已正确安装，并配置好环境变量。
- 仿真环境中的传感器和模型配置可能需要根据实际情况进行调整。

希望这个项目能帮助你快速上手四轮机器人的仿真！如果有任何问题或建议，请随时联系我。

