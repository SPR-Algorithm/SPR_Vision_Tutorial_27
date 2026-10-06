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
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(io)
add_subdirectory(tasks)
add_subdirectory(tools)

add_executable(main main.cpp)
add_executable(example io/example.cpp)

add_custom_command(TARGET main POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_directory
${CMAKE_CURRENT_SOURCE_DIR}/configs
$<TARGET_FILE_DIR:main>/configs
)
add_custom_command(TARGET main POST_BUILD
COMMAND ${CMAKE_COMMAND} -E copy_directory
${CMAKE_CURRENT_SOURCE_DIR}/assets
$<TARGET_FILE_DIR:main>/assets
)
target_link_libraries(main ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(example ${OpenCV_LIBS} tools io)
