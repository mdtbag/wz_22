cmake_minimum_required(VERSION 3.8)
project(cost)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

aux_source_directory(${PROJECT_SOURCE_DIR}/src MAIN_SRC)

include_directories(${PROJECT_SOURCE_DIR}/include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

add_executable(cost ${MAIN_SRC})

