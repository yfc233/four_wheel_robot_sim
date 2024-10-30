cmake_minimum_required(VERSION 3.5)
project(four_wheel_robot_gazebo)

find_package(ament_cmake REQUIRED)

# 安装launch文件和worlds文件夹
install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()