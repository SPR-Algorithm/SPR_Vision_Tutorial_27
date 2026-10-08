cmake_minimum_required(VERSION 3.16)
project(homework1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    
add_library(vector STATIC src/vector.cpp)
target_include_directories(vector PUBLIC include)

add_executable(test_main test/main.cpp)
target_link_libraries(test_main PRIVATE vector)