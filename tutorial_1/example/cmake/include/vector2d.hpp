// CMake 最小构建示例 —— 头文件（声明）
// 和 example/gpp/multi 里的同一份代码，区别只在于编译方式：
// 那边手写 g++ 命令，这边交给 CMakeLists.txt 描述。

#pragma once

namespace rm {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

double length(const Vec2 &v);                  // 模长
double distance(const Vec2 &a, const Vec2 &b); // 两点距离
double dot(const Vec2 &a, const Vec2 &b);      // 点乘
Vec2 scale(const Vec2 &v, double k);           // 缩放

} // namespace rm
