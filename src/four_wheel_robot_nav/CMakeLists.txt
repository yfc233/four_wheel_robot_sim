cmake_minimum_required(VERSION 3.5)
project(four_wheel_robot_nav)

# 添加编译选项
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找构建工具依赖
find_package(ament_cmake REQUIRED)

# 查找构建依赖
find_package(nav2_common REQUIRED)

# 查找构建和执行依赖
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)

# 安装目录
install(DIRECTORY
  launch
  config
  worlds
  behavior_trees
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()