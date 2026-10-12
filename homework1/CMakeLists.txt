cmake_minimum_required(VERSION 3.16)
project(hw1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HW1_WARNINGS "")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(HW1_WARNINGS -Wall -Wextra)
endif()

# ---- 库：作业要实现的就是它 ----
add_library(rm_vector STATIC src/vector.cpp)
target_include_directories(rm_vector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(rm_vector PRIVATE ${HW1_WARNINGS})
