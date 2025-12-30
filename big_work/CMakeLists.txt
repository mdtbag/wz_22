cmake_minimum_required(VERSION 3.8)
project(bigwork)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

aux_source_directory(src/ MAIN)

find_package(OpenCV REQUIRED)

include_directories(include)

foreach(source_file ${MAIN})
  get_filename_component(target_name ${source_file} NAME_WE)
  add_executable(${target_name} ${source_file})
  target_link_libraries(${target_name} ${OpenCV_LIBS})
  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()


install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/
)

