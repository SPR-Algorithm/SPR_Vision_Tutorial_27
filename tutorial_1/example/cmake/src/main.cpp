// CMake 最小构建示例 —— 调用方
//
// 上课时执行：
//   cd example/cmake
//   cmake -S . -B build && cmake --build build
//   ./build/vec2_demo
//
// 输出：
//   |a|        = 5
//   dist(a,b)  = 5
//   a·b        = 0
//   2a         = (6, 8)
//
// 注意这里没有写任何 -I 路径 —— 因为 CMakeLists.txt 里
// target_include_directories(vec2 PUBLIC ...) 已经把 include 目录传过来了。

#include <iostream>

#include "vector2d.hpp"

int main() {
  const rm::Vec2 a{3.0, 4.0};
  const rm::Vec2 b{0.0, 0.0};

  std::cout << "|a|        = " << rm::length(a) << '\n';
  std::cout << "dist(a,b)  = " << rm::distance(a, b) << '\n';
  std::cout << "a·b        = " << rm::dot(a, b) << '\n';

  const rm::Vec2 c = rm::scale(a, 2.0);
  std::cout << "2a         = (" << c.x << ", " << c.y << ")\n";

  return 0;
}
