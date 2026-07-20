cmake_minimum_required(VERSION 3.8)
project(ur_start)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

# 编译节点
add_executable(ur_start_node ur_start.cpp)

ament_target_dependencies(
  ur_start_node
  rclcpp
  ur_dashboard_msgs
)

# 安装
install(TARGETS
  ur_start_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
