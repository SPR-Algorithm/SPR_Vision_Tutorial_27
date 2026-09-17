// g++ 编译示例 ② · 多文件 —— 头文件只放「声明」
//
// 头文件 = 接口说明书：只告诉别人「有哪些函数、什么参数、返回什么」，
//          函数体全部写在 src/vector2d.cpp 里。
//
// #pragma once 用来防止同一个头文件被重复展开，否则会 multiple definition。

#pragma once

namespace rm {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

double length(const Vec2 &v);                  // 模长：|v| = sqrt(x² + y²)
double distance(const Vec2 &a, const Vec2 &b); // 两点距离：|a - b|
double dot(const Vec2 &a, const Vec2 &b);      // 点乘：a.x*b.x + a.y*b.y
Vec2 scale(const Vec2 &v, double k);           // 缩放：返回 k * v

} // namespace rm
