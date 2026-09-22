cmake_minimum_required(VERSION 3.16)
project(vector_math)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 这里必须明确告诉CMake要编译哪个源文件
add_library(vector_math STATIC
    src/vector_math.cpp
)

target_include_directories(vector_math PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
