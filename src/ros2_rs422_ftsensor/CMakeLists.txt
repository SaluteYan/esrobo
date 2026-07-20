cmake_minimum_required(VERSION 3.8)
project(kw_ft_sensor_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Network)

set(CMAKE_AUTOMOC ON)              # 自动运行 MOC 处理 Q_OBJECT 类
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 包含头文件目录
include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Network_INCLUDE_DIRS}
)

# 源文件列表
set(SOURCES
  src/FTTcpDriver.cpp
  src/FTSerialDriver.cpp
  src/WebFTSensorAdapterCobot.cpp
  src/FTTcpDriverCtl.hpp
  src/kw_ft_node.cpp
)

# 生成可执行文件
add_executable(kw_ft_sensor_node ${SOURCES})

# 链接依赖库
ament_target_dependencies(kw_ft_sensor_node
  rclcpp
  geometry_msgs
  std_msgs
)

target_link_libraries(kw_ft_sensor_node
  Qt5::Core
  Qt5::Network
)

# 安装可执行文件
install(TARGETS
  kw_ft_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
