cmake_minimum_required(VERSION 3.8)
project(px4_gen_trajectory)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gen_trajectory_node
  src/gen_trajectory_node.cpp)

ament_target_dependencies(gen_trajectory_node
  rclcpp px4_msgs geometry_msgs std_msgs)

install(TARGETS gen_trajectory_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
