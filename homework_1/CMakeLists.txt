cmake_minimum_required(VERSION 3.20)
project(homework_1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_library(rm_vector STATIC src/vector.cpp)
target_include_directories(rm_vector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(rm_matrix STATIC src/matrix.cpp)
target_link_libraries(rm_matrix PUBLIC rm_vector)

add_executable(test_vector test/test_vector.cpp)
target_link_libraries(test_vector PRIVATE rm_vector)

add_executable(test_matrix test/test_matrix.cpp)
target_link_libraries(test_matrix PRIVATE rm_matrix)