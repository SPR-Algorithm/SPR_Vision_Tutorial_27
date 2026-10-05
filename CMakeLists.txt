cmake_minimum_required(VERSION 3.16)
project(rm_vector_matrix_demo LANGUAGES CXX)

# 引入FetchContent模块，用于拉取GTest
include(FetchContent)

# 构建 vector2d 静态库 rm_vector
add_library(rm_vector
    src/vector2d.cpp
)
target_include_directories(rm_vector PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 构建 matrix 静态库 matrix_lib
add_library(matrix_lib
    src/matrix.cpp
)
target_include_directories(matrix_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 主程序 main，链接上面两个库
add_executable(main
    src/main.cpp
)
target_link_libraries(main PRIVATE
    rm_vector
    matrix_lib
)

# ========== GTest 配置部分 ==========
# 优先找系统已经装好的GTest，找不到再触发下载
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    message(STATUS "未找到系统GTest，将通过FetchContent下载编译 googletest")
    # 设置Windows CRT库兼容，避免MSVC编译冲突
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
        # 增加配置，解决部分网络场景的超时，如网络很差优先用下面的本地方案
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
    
    FetchContent_MakeAvailable(googletest)
endif()

# 开启测试
enable_testing()

# 构建测试可执行文件（注意路径，你的测试代码放在 test/test_matrix.cpp）
add_executable(test_matrix
    test/test_matrix.cpp
)
target_link_libraries(test_matrix PRIVATE
    rm_vector
    matrix_lib
    GTest::gtest_main
)

# 注册测试
include(GoogleTest)
gtest_discover_tests(test_matrix)