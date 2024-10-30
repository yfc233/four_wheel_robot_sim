cmake_minimum_required(VERSION 3.5)
project(four_wheel_robot_navigation)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_common REQUIRED)

# 安装launch和config文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 确保所有依赖项都被正确处理
ament_package()