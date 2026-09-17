# =============================================================================
# 知识点 2 · OpenCV 装甲板识别
# 对应教案：二、OpenCV（2.2 环境搭建与最小 CMake 工程）
#
# 上课时执行：
#   cd example/opencv
#   cmake -S . -B build
#   cmake --build build
#   ./build/armor_detect              # 用内置合成测试图
#
# 和第一阶段相比，这里只多了一件事：find_package(OpenCV)
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(armor_detect LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 找 OpenCV：REQUIRED 表示找不到就直接报错，而不是编到一半才炸
find_package(OpenCV REQUIRED)

add_executable(armor_detect armor_detect.cpp)

# 头文件目录和库都要手动接上
target_include_directories(armor_detect PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(armor_detect PRIVATE ${OpenCV_LIBS})

target_compile_options(armor_detect PRIVATE -Wall -Wextra)

message(STATUS "OpenCV 版本：${OpenCV_VERSION}")
message(STATUS "OpenCV 库文件：${OpenCV_LIBS}")
