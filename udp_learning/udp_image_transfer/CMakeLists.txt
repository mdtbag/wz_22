cmake_minimum_required(VERSION 3.10)
project(UDPImageTransfer)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV库
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found! Please install OpenCV first.")
endif()

# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 编译服务器端
add_executable(udp_image_server server.cpp)
target_link_libraries(udp_image_server ${OpenCV_LIBS} pthread)

# 编译客户端
add_executable(udp_image_client client.cpp)
target_link_libraries(udp_image_client ${OpenCV_LIBS} pthread)