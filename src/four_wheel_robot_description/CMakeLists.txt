cmake_minimum_required(VERSION 3.5)
project(four_wheel_robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# 确保URDF文件被正确安装
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 