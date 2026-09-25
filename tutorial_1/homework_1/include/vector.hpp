// rm::vector —— 简单的 n 维向量运算库
//
// 边界约定（与 test/test_vector.cpp 一致）：
//   * 维度不一致            -> 抛 std::logic_error（这里用 std::invalid_argument）
//   * 元素 / 标量非有限     -> 抛 std::logic_error（这里用 std::domain_error）
//   * 零向量（含近零向量）单位化 -> 返回同维度的全 0 向量，绝不除零
//   * 零向量参与夹角        -> 约定夹角为 0
//   * 空向量：length/dot/distance/angleBetween 为 0，normalize/scale 仍为空
//   * 全程不产生 NaN / inf

#pragma once

#include <vector>

namespace rm {

using Vector = std::vector<double>;                // n 维向量，二维就是 size() == 2
using Matrix = std::vector<std::vector<double>>;   // 矩阵

// ||v|| —— 向量模长。空向量与零向量返回 0。
double length(const Vector& v);

// ||a - b|| —— 两点间距离。
double distance(const Vector& a, const Vector& b);

// a · b —— 点乘。
double dot(const Vector& a, const Vector& b);

// k * v —— 缩放。
Vector scale(const Vector& v, double k);

// v / ||v|| —— 单位化。零向量（模长低于阈值）返回同维度的全 0 向量。
Vector normalize(const Vector& v);

// a 与 b 的夹角，范围 [0, pi]。任一为零向量时返回 0。
double angleBetween(const Vector& a, const Vector& b);

}  // namespace rm
