cmake_minimum_required(VERSION 3.16.3)

project(homework_2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# 生成 build/compile_commands.json，供 clangd / cpptools 做代码跳转与检索
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/usr/lib/cmake/openvino2024.6.0/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(io)
add_subdirectory(tasks)
add_subdirectory(tools)

add_executable(main main.cpp)
add_executable(example io/example.cpp)

target_link_libraries(main ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(example ${OpenCV_LIBS} tools io)

# 演示目标：读摄像头 -> 识别 -> 存图（不用 imshow，headless 可跑）
add_executable(demo demo.cpp)
target_link_libraries(demo ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

# 读视频跑识别的演示（headless 存图，不影响 main/example/demo）
add_executable(video_demo video_demo.cpp)
target_link_libraries(video_demo ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
