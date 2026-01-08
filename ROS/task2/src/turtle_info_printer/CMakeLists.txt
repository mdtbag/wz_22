cmake_minimum_required(VERSION 3.8)
project(turtle_info_printer)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_info_printer_node src/turtle_info_printer_node.cpp)
ament_target_dependencies(turtle_info_printer_node rclcpp turtlesim)

install(TARGETS turtle_info_printer_node DESTINATION lib/${PROJECT_NAME})

install(
  FILES package.xml DESTINATION share/${PROJECT_NAME}
)

ament_package()
