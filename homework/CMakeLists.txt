cmake_minimum_required(VERSION 3.16)
project(vec2_demo LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(vec2 STATIC src/fax1.cpp)
target_include_directories(vec2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(vec2_demo src/main.cpp)
target_link_libraries(vec2_demo PRIVATE vec2)

