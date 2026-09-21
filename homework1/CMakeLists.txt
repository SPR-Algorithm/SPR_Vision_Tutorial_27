cmake_minimum_required(VERSION 3.10)
project(homework1)
aux_source_directory (src SRC_LIST)
add_library(Vector2 STATIC ${SRC_LIST})
target_include_directories(Vector2 PUBLIC include) 
include_directories(include)
add_executable(Vec ${SRC_LIST})

