cmake_minimum_required(VERSION 3.8)
project(px4_trajectory_visualizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(trajectory_visualizer_node
  src/trajectory_visualizer_node.cpp)

ament_target_dependencies(trajectory_visualizer_node
  rclcpp px4_msgs nav_msgs geometry_msgs tf2_ros visualization_msgs)

install(TARGETS trajectory_visualizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
