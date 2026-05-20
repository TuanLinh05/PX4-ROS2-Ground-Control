cmake_minimum_required(VERSION 3.8)
project(px4_lqr_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(lqr_controller_node
  src/lqr_controller_node.cpp)

target_include_directories(lqr_controller_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(lqr_controller_node
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs)

install(TARGETS lqr_controller_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
