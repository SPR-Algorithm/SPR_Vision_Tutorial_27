#pragma once
#include <vector>

namespace rm {
using Vector = std::vector<double>;
using Matrix = std::vector<std::vector<double>>;

// 向量模长
double length(const Vector& v);
// 两个向量距离
double distance(const Vector& a, const Vector& b);
// 向量点乘
double dot(const Vector& a, const Vector& b);
// 向量缩放
Vector scale(const Vector& v, double k);
// 向量单位化
Vector normalize(const Vector& v);
// 向量夹角（弧度）
double angleBetween(const Vector& a, const Vector& b);
}