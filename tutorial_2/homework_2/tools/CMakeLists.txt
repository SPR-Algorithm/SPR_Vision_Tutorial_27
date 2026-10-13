cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT
    img_tools.cpp
    logger.cpp
)

# logger.cpp 用到 fmt 和 spdlog
target_link_libraries(tools PUBLIC fmt::fmt spdlog::spdlog)