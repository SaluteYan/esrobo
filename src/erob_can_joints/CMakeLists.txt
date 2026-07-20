cmake_minimum_required(VERSION 3.8)
project(erob_canopen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(erob_driver_node
  erob_driver_node.cpp
)

ament_target_dependencies(erob_driver_node
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(erob_driver_node pthread)

install(TARGETS
  erob_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
