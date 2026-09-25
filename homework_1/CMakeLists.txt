set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.16)
project(homework_1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 把 vector.cpp 编译成静态库
add_library(vector STATIC src/vector.cpp)

# 把 include 目录暴露出去，别人包含这个库的时候能找到头文件
target_include_directories(vector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
