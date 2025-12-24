cmake_minimum_required(VERSION 3.5)
project(robot_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 接口生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LidarData.msg"
  "srv/RobotStateSrv.srv"
  "action/MoveAction.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_executable(lidar_publisher src/lidar_publisher.cpp)
ament_target_dependencies(lidar_publisher
  rclcpp rcl_interfaces std_msgs)

add_executable(obstacle_detector src/obstacle_detector.cpp)
ament_target_dependencies(obstacle_detector
  rclcpp rcl_interfaces std_msgs)

add_executable(robot_service src/robot_service.cpp)
ament_target_dependencies(robot_service
  rclcpp rcl_interfaces std_srvs)

add_executable(control_client src/control_client.cpp)
ament_target_dependencies(control_client
  rclcpp std_srvs)

add_executable(move_action_server src/move_action_server.cpp)
ament_target_dependencies(move_action_server
  rclcpp rclcpp_action rcl_interfaces std_msgs)

add_executable(move_action_client src/move_action_client.cpp)
ament_target_dependencies(move_action_client
  rclcpp rclcpp_action)

rosidl_get_typesupport_target(cpp_ts_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(lidar_publisher ${cpp_ts_target})
target_link_libraries(obstacle_detector ${cpp_ts_target})
target_link_libraries(robot_service ${cpp_ts_target})
target_link_libraries(control_client ${cpp_ts_target})
target_link_libraries(move_action_server ${cpp_ts_target})
target_link_libraries(move_action_client ${cpp_ts_target})

install(TARGETS
  lidar_publisher
  obstacle_detector
  robot_service
  control_client
  move_action_server
  move_action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  builtin_interfaces
  rcl_interfaces
  rosidl_default_runtime
)

ament_package()
