cmake_minimum_required(VERSION 3.5)
project(four_wheel_robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()