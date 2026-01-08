cmake_minimum_required(VERSION 3.8)
project(hahab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Wts.srv"
)

add_executable(hahab_i src/hahab_i.cpp)
ament_target_dependencies(hahab_i rclcpp)

add_executable(hahab_s src/hahab_s.cpp)
ament_target_dependencies(hahab_s rclcpp)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(hahab_i ${cpp_typesupport_target})
target_link_libraries(hahab_s ${cpp_typesupport_target})

install(TARGETS 
  hahab_i
  hahab_s
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()

