cmake_minimum_required(VERSION 3.16)
project(homework_1  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(rm_vector STATIC src/vector.cpp)
target_include_directories(rm_vector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_options(vector PRIVATE -Wall -Wextra)
