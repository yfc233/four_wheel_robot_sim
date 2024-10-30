cmake_minimum_required(VERSION 3.5)
project(four_wheel_robot_description)

find_package(ament_cmake REQUIRED)
find_package(image_transport_plugins REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)

# 安装launch文件
install(DIRECTORY
  launch
  urdf
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 