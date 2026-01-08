cmake_minimum_required(VERSION 3.8)
project(hahac)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Wts.action"
)

add_executable(hahac_i src/hahac_i.cpp)
ament_target_dependencies(hahac_i rclcpp rclcpp_action)

add_executable(hahac_s src/hahac_s.cpp)
ament_target_dependencies(hahac_s rclcpp rclcpp_action)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(hahac_i ${cpp_typesupport_target})
target_link_libraries(hahac_s ${cpp_typesupport_target})

install(TARGETS 
  hahac_i
  hahac_s
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()

