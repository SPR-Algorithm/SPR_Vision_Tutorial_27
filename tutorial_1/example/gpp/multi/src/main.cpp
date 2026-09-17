// g++ 编译示例 ② · 多文件 —— 调用方
//
// 上课时执行：
//   cd example/gpp/multi
//   g++ -std=c++17 -Wall -Wextra -Iinclude src/main.cpp src/vector2d.cpp -o app
//   ./app
//
// 输出：
//   |a|        = 5
//   dist(a,b)  = 5
//   a·b        = 0
//   2a         = (6, 8)
//
// 记忆点：-Iinclude 让编译器找到【声明】；
//         把 src/vector2d.cpp 一起编，链接器才找得到【实现】。

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
