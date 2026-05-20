cmake_minimum_required(VERSION 3.8)
project(px4_test_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(include)

add_executable(px4_bridge_node src/px4_bridge_node.cpp)
ament_target_dependencies(px4_bridge_node rclcpp px4_msgs)

install(TARGETS px4_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
