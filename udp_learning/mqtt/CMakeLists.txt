cmake_minimum_required(VERSION 3.10)
project(MQTT_Protobuf_Time)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# 查找Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

# 查找Mosquitto
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
include_directories(${MOSQUITTO_INCLUDE_DIRS})

# 包含当前目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 生成的Protobuf代码（先通过protoc生成，CMake会自动识别）
set(PROTO_SRCS time_message.pb.cc)
set(PROTO_HDRS time_message.pb.h)

# 公共工具代码
set(COMMON_SRCS mqtt_protobuf_common.cpp)
set(COMMON_HDRS mqtt_protobuf_common.h)

# 编译发布者
add_executable(mqtt_publisher 
    mqtt_publisher.cpp
    ${COMMON_SRCS}
    ${PROTO_SRCS}
)
target_link_libraries(mqtt_publisher
    ${PROTOBUF_LIBRARIES}
    ${MOSQUITTO_LIBRARIES}
    pthread
)

# 编译订阅者
add_executable(mqtt_subscriber
    mqtt_subscriber.cpp
    ${COMMON_SRCS}
    ${PROTO_SRCS}
)
target_link_libraries(mqtt_subscriber
    ${PROTOBUF_LIBRARIES}
    ${MOSQUITTO_LIBRARIES}
    pthread
)

# 编译优化
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mqtt_publisher PRIVATE -O3 -Wall -Wno-deprecated-declarations)
    target_compile_options(mqtt_subscriber PRIVATE -O3 -Wall -Wno-deprecated-declarations)
endif()