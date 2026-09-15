cmake_minimum_required(VERSION 3.8)
project(jacobian_checker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(jacobian_checker
  src/jacobian_checker.cpp
)

ament_target_dependencies(jacobian_checker
  rclcpp
  moveit_ros_planning_interface
  moveit_core
)

install(TARGETS
  jacobian_checker
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
