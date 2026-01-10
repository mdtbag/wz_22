cmake_minimum_required(VERSION 3.10)
project(mqtt_cpp_demo)

# 升级到 C++17（兼容 JSON 库和 Paho 库）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 增加编译选项，兼容新库特性
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -DPAHO_MQTT_CPP_NO_DEPRECATED")

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 关键：添加 include 目录，让编译器找到 json.hpp
include_directories(${CMAKE_SOURCE_DIR}/include)

# 查找 Paho 库（兼容新旧版本）
find_library(PAHO_MQTT_CPP NAMES paho-mqttpp3 REQUIRED)
find_library(PAHO_MQTT_C NAMES paho-mqtt3as paho-mqtt3a REQUIRED)

# 查找线程库和 SSL 库
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

# 编译订阅端（保留原逻辑，可用于接收 JSON 消息）
add_executable(subscriber subscriber.cpp)
target_link_libraries(subscriber
    ${PAHO_MQTT_CPP}
    ${PAHO_MQTT_C}
    Threads::Threads
    OpenSSL::SSL
    OpenSSL::Crypto
)

# 编译发布端（新增 JSON 依赖）
add_executable(publisher publisher.cpp)
target_link_libraries(publisher
    ${PAHO_MQTT_CPP}
    ${PAHO_MQTT_C}
    Threads::Threads
    OpenSSL::SSL
    OpenSSL::Crypto
)

# 打印编译信息
message(STATUS "Paho MQTT C++ 库路径: ${PAHO_MQTT_CPP}")
message(STATUS "Paho MQTT C 库路径: ${PAHO_MQTT_C}")
message(STATUS "可执行文件输出目录: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "JSON 头文件路径: ${CMAKE_SOURCE_DIR}/include/json.hpp")