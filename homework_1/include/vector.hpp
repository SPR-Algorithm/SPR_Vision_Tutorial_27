#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <vector>

namespace rm {

using Vector = std::vector<double>;

// 计算向量的模长
double length(const Vector& v);

// 计算两个向量之间的欧氏距离
double distance(const Vector& a, const Vector& b);

// 计算两个向量的点乘
double dot(const Vector& a, const Vector& b);

// 向量乘以常数 k
Vector scale(const Vector& v, double k);

// 单位化（变成长度为 1 的向量）
Vector normalize(const Vector& v);

// 计算两个向量的夹角（返回弧度值）
double angleBetween(const Vector& a, const Vector& b);

} // namespace rm

#endif // VECTOR_HPP
