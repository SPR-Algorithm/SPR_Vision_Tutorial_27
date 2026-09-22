# =============================================================================
# 第一阶段作业 · 标准答案 + 自动化测试
# 对应作业：tutorial_1/homework_1.md
#
#   cmake -S . -B build
#   cmake --build build
#   cd build && ctest --output-on-failure
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(hw1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HW1_WARNINGS "")
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(HW1_WARNINGS -Wall -Wextra)
endif()

# ---- 库：作业要实现的就是它 ----
add_library(rm_vector STATIC src/vector.cpp)
target_include_directories(rm_vector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(rm_vector PRIVATE ${HW1_WARNINGS})

# ---- 自动化测试 ----
enable_testing()

# 优先用系统装好的 GTest；没有就自动下载（需要网络）
find_package(GTest QUIET)
if(NOT GTest_FOUND)
  message(STATUS "未找到系统 GTest，改用 FetchContent 下载 googletest")
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
else()
  message(STATUS "使用系统的 GTest")
endif()

add_executable(test_vector test/test_vector.cpp)
# GTest::gtest_main 已经带了 GTest::gtest，不用再单独写一遍
target_link_libraries(test_vector PRIVATE rm_vector GTest::gtest_main)
target_compile_options(test_vector PRIVATE ${HW1_WARNINGS})

add_test(NAME test_vector COMMAND test_vector)


