# =============================================================================
# CMake 最小构建示例
#
# 上课时执行：
#   cd example/cmake
#   cmake -S . -B build       # 配置：读取本文件，在 build/ 生成构建文件
#   cmake --build build       # 构建：调用 make 真正编译
#   ./build/vec2_demo         # 运行
#
# -S 是源码目录，-B 是构建目录。永远不要把中间文件吐在源码目录里。
# =============================================================================

cmake_minimum_required(VERSION 3.16)      # ① CMake 最低版本
project(vec2_demo LANGUAGES CXX)          # ② 项目名 + 语言

set(CMAKE_CXX_STANDARD 17)                # ③ 用 C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ④ 把 vector2d.cpp 编成静态库目标 vec2
#    add_library = 「我要一段能被复用的代码」
add_library(vec2 STATIC src/vector2d.cpp)

#    PUBLIC：这个头文件目录既给 vec2 自己用，也传给所有链接 vec2 的目标，
#    所以下面的 vec2_demo 不用再写一遍 include 目录。
#    改成 PRIVATE，vec2_demo 里 #include "vector2d.hpp" 会立刻报
#    fatal error: 'vector2d.hpp' file not found
target_include_directories(vec2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ⑤ 把 main.cpp 编成可执行文件目标，并链接上面那个库
#    add_executable        = 「我要一个能被运行的产物」
#    target_link_libraries = 「A 要用 B」
add_executable(vec2_demo src/main.cpp)
target_link_libraries(vec2_demo PRIVATE vec2)
