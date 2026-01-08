cmake_minimum_required(VERSION 3.15)
project(task1)

aux_source_directory(../src MAIN_SRC)

include_directories(/usr/include/eigen3)
include_directories(../include)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

add_executable(task1 ${MAIN_SRC})

install(TARGETS
	task1
	DESTINATION lib/${PROJECT_NAME})
