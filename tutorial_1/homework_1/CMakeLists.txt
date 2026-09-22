cmake_minimum_requird(VERSION 3.16)
project(homework_1 LANGUAGES CXX)
add_library(lib scr/vector.cpp)
target_include_directories(lib PUBLIC include/)

