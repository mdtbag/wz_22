# 最低版本号
cmake_minimum_required(VERSION 3.0)

# 项目名称
project(learn_lib)

# 整合源文件
aux_source_directory(${PROJECT_BINARY_DIR}/../src SRC_LIST)

# 引入头文件路径
include_directories(${PROJECT_BINARY_DIR}/../inc)

# 生成静态库或动态库，参数一：生成的库的名称；参数二：静态或动态；参数三：生成所需要的源文件
add_library(func_shared SHARED ${SRC_LIST})
add_library(func_static STATIC ${SRC_LIST})

# 设置最终生成的库名称，这里把库名称统一为myfunc
set_target_properties(func_shared PROPERTIES OUTPUT_NAME "myfunc")
set_target_properties(func_static PROPERTIES OUTPUT_NAME "myfunc")

# 设置库默认生成到哪里
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../lib)