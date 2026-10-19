# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)      # ① 声明 CMake 最低版本
project(vec2_demo LANGUAGES CXX)          # ② 项目名 + 语言

set(CMAKE_CXX_STANDARD 17)                # ③ 用 C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)     # 生成 compile_commands.json，给 VS Code 补全用

# ④ 把 vector2d.cpp 编译成一个静态库目标 vec2
add_library(vec2 STATIC src/vector2d.cpp)
target_include_directories(vec2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ⑤ 把 main.cpp 编译成可执行文件目标，并链接上面那个库
add_executable(vec2_demo src/main.cpp)
target_link_libraries(vec2_demo PRIVATE vec2)

target_compile_options(vec2 PRIVATE -Wall -Wextra)
