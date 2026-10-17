cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT
    img_tools.cpp
    logger.cpp
)

# 头文件对上层可见（yaml.hpp 等直接 include "tools/logger.hpp"）
target_include_directories(tools PUBLIC ${PROJECT_SOURCE_DIR})

# logger.cpp 用到 spdlog + fmt；img_tools.cpp 用到 OpenCV
target_link_libraries(tools PUBLIC spdlog::spdlog fmt::fmt ${OpenCV_LIBS})