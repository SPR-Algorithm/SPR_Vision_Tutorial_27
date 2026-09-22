#pragma once
#include <vector>

namespace rm {

using Vector = std::vector<double>;
using Matrix = std::vector<std::vector<double>>;

// 计算向量模长
double length(const Vector& v);

// 计算两个向量的欧氏距离
double distance(const Vector& a, const Vector& b);

// 计算两个向量的点乘结果
double dot(const Vector& a, const Vector& b);

// 向量缩放k倍
Vector scale(const Vector& v, double k);

// 向量单位化（归一化）
Vector normalize(const Vector& v);

// 计算两个向量的夹角（返回弧度值）
double angleBetween(const Vector& a, const Vector& b);

} // namespace rm
