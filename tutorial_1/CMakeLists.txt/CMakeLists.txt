cmake_minimum_required(VERSION 3.16)
project(vector_math)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 明确指定源文件的完整路径
add_library(vector_math STATIC
    ${PROJECT_SOURCE_DIR}/src/vector_math.cpp
)

target_include_directories(vector_math PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
