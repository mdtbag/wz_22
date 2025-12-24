cmake_minimum_required(VERSION 3.8)
project(turtle_square)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(turtle_square_node src/turtle_square_node.cpp)
ament_target_dependencies(turtle_square_node rclcpp geometry_msgs turtlesim std_srvs)

add_executable(turtle_circle src/turtle_circle.cpp)
ament_target_dependencies(turtle_circle rclcpp geometry_msgs turtlesim std_srvs)

install(TARGETS 
  turtle_square_node 
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS 
  turtle_circle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
