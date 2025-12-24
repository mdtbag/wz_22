cmake_minimum_required(VERSION 3.8)
project(turtle_chase)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_chase_node src/turtle_chase_node.cpp)
ament_target_dependencies(turtle_chase_node rclcpp geometry_msgs turtlesim)

add_executable(turtle_escape src/turtle_escape.cpp)
ament_target_dependencies(turtle_escape rclcpp geometry_msgs turtlesim)

install(TARGETS turtle_chase_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  turtle_escape
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
