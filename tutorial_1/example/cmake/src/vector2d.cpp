// CMake 最小构建示例 —— 源文件（实现）
//
// 这个文件被 add_library(vec2 STATIC src/vector2d.cpp) 编进静态库 libvec2.a。
// 上课时可以演示增量编译：只碰一下这个文件，看 make 只重编它、不重编 main.cpp
//   touch src/vector2d.cpp && cmake --build build

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
