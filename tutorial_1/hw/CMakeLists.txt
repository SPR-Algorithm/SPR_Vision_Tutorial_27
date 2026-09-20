# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)  # ① 声明 CMake 最低版本
project(qvec_demo LANGUAGES CXX)      # ② 项目名 + 语言

set(CMAKE_CXX_STANDARD 17)            # ③ 用 C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # 生成 compile_commands.json，给 VS Code 补全用

add_library(qvec STATIC src/qvec.cpp)
target_include_directories(qvec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(qvec_demo src/main.cpp)
target_link_libraries(qvec_demo PRIVATE qvec)

add_executable(qvec_test test/test_qvec.cpp)
target_link_libraries(qvec_test PRIVATE qvec)

enable_testing()
add_test(NAME qvec_test COMMAND qvec_test)