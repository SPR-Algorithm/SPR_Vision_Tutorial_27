cmake_minimum_required(VERSION 3.16)

project(homework_1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- 向量运算静态库 ----
add_library(rm_vector STATIC src/vector.cpp)

# 把 include/ 暴露给使用者：他们只写 #include "vector.hpp"
target_include_directories(rm_vector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rm_vector PRIVATE -Wall -Wextra -Wpedantic)
endif()