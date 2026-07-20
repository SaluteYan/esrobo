
cmake_minimum_required(VERSION 3.8)
project(zero_diff_canopen_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_402_driver REQUIRED)
find_package(canopen_ros2_control REQUIRED)

# 安装配置文件和启动文件
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
