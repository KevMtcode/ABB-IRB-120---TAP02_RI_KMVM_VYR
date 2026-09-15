cmake_minimum_required(VERSION 3.8)

project(irb120_cartesian)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(
  cartesian_path
  src/cartesian_path.cpp
)

ament_target_dependencies(
  cartesian_path
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  trajectory_msgs
)

install(
  TARGETS cartesian_path
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
add_executable(
  named_move
  src/named_move.cpp
)

ament_target_dependencies(
  named_move
  rclcpp
  moveit_ros_planning_interface
)

install(
  TARGETS named_move
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
