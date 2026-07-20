cmake_minimum_required(VERSION 3.8)
project(slam_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)

# 安装 launch 启动文件到系统目录
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 测试配置（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

