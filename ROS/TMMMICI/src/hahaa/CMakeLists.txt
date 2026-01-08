cmake_minimum_required(VERSION 3.8)
project(hahaa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Wts.msg"
  "msg/So.msg"
)

add_executable(hahaa_i src/hahaa_i.cpp)
ament_target_dependencies(hahaa_i rclcpp)

add_executable(hahaa_s src/hahaa_s.cpp)
ament_target_dependencies(hahaa_s rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(hahaa_i ${cpp_typesupport_target})
target_link_libraries(hahaa_s ${cpp_typesupport_target})

install(TARGETS 
  hahaa_i
  hahaa_s
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()

