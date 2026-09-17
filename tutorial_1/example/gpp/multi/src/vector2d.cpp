// g++ 编译示例 ② · 多文件 —— 源文件放「实现」
//
// 这个文件必须参与编译，否则 main.cpp 里的调用会在【链接】阶段报：
//
// 上课对照（同一份代码，一条命令之差）：
// g++ -std=c++17 -Iinclude src/main.cpp src/vector2d.cpp -o app
// g++ -std=c++17 -Iinclude src/main.cpp -o app

#include "vector2d.hpp"

#include <cmath>

namespace rm {

double length(const Vec2 &v) { return std::sqrt(v.x * v.x + v.y * v.y); }

double distance(const Vec2 &a, const Vec2 &b) {
  return length(Vec2{a.x - b.x, a.y - b.y});
}

double dot(const Vec2 &a, const Vec2 &b) { return a.x * b.x + a.y * b.y; }

Vec2 scale(const Vec2 &v, double k) { return Vec2{v.x * k, v.y * k}; }

} // namespace rm
