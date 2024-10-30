cmake_minimum_required(VERSION 3.5)
project(four_wheel_robot_description)

find_package(ament_cmake REQUIRED)

# 安装launch文件
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 