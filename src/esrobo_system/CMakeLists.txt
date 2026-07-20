cmake_minimum_required(VERSION 3.8)
project(esrobo_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 1. 安装launch文件夹
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. 关键：安装package.xml到安装目录，ros2 pkg才能识别包
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
