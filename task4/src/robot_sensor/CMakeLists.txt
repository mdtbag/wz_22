cmake_minimum_required(VERSION 3.8)
project(robot_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(robot_sensor_node src/robot_sensor_node.cpp)

ament_target_dependencies(robot_sensor_node
  rclcpp
  geometry_msgs
  tf2_ros
)

install(TARGETS robot_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(DIRECTORY launch/
	DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
