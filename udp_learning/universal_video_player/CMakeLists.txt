cmake_minimum_required(VERSION 3.10)
project(UniversalVideoPlayer)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV库
find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "未找到OpenCV库，请先安装！")
endif()

# 打印OpenCV信息（调试用）
message(STATUS "OpenCV版本: ${OpenCV_VERSION}")
message(STATUS "OpenCV包含目录: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV库文件: ${OpenCV_LIBS}")

# 包含头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})

# 源文件列表
set(SOURCES
    main.cpp
    VideoPlayer.cpp
    VideoPlayer.h
)

# 生成可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接OpenCV库
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})