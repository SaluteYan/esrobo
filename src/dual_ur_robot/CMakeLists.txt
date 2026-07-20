cmake_minimum_required(VERSION 3.8)
project(dual_ur_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------
# 安装 Launch 文件
# ---------------------------
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ---------------------------
# 最后调用 ament_package()
# ---------------------------
ament_package()
