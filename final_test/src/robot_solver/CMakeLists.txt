cmake_minimum_required(VERSION 3.8)
project(robot_solver)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(solver_node src/solver_node.cpp)

ament_target_dependencies(solver_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  robot_interfaces
)

install(TARGETS solver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

