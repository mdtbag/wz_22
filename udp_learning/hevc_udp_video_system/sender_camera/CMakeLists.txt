cmake_minimum_required(VERSION 3.19)
project(HEVC_UDP_Camera_Sender LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVFORMAT REQUIRED libavformat)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)

find_package(OpenCV REQUIRED)

# 源文件
add_executable(HEVC_UDP_Camera_Sender
    hevc_udp_camera_sender.cpp
)

# 链接库
target_link_libraries(HEVC_UDP_Camera_Sender
    PRIVATE
        ${AVCODEC_LIBRARIES}
        ${AVFORMAT_LIBRARIES}
        ${AVUTIL_LIBRARIES}
        ${SWSCALE_LIBRARIES}
        ${OpenCV_LIBS}
        pthread
)

# 包含目录
target_include_directories(HEVC_UDP_Camera_Sender
    PRIVATE
        ${AVCODEC_INCLUDE_DIRS}
        ${AVFORMAT_INCLUDE_DIRS}
        ${AVUTIL_INCLUDE_DIRS}
        ${SWSCALE_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# 编译选项
target_compile_options(HEVC_UDP_Camera_Sender
    PRIVATE
        -Wall
        -O2
)