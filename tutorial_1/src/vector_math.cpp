#include "vector_math.h"
#include <cmath>
#include <stdexcept>

namespace rm {

double length(const Vector& v) {
    // 空向量边界处理
    if (v.empty()) {
        throw std::invalid_argument("输入向量不能为空");
    }
    double sum_sq = 0.0;
    for (double num : v) {
        sum_sq += num * num;
    }
    return std::sqrt(sum_sq);
}

double distance(const Vector& a, const Vector& b) {
    // 校验两个向量维度一致
    if (a.size() != b.size()) {
        throw std::invalid_argument("参与运算的向量维度必须一致");
    }
    // 空向量边界处理
    if (a.empty()) {
        throw std::invalid_argument("输入向量不能为空");
    }
    double sum_sq = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        double diff = a[i] - b[i];
        sum_sq += diff * diff;
    }
    return std::sqrt(sum_sq);
}

double dot(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        throw std::invalid_argument("参与运算的向量维度必须一致");
    }
    if (a.empty()) {
        throw std::invalid_argument("输入向量不能为空");
    }
    double result = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        result += a[i] * b[i];
    }
    return result;
}

Vector scale(const Vector& v, double k) {
    if (v.empty()) {
        throw std::invalid_argument("输入向量不能为空");
    }
    Vector res;
    res.reserve(v.size());
    for (double num : v) {
        res.push_back(num * k);
    }
    return res;
}

Vector normalize(const Vector& v) {
    if (v.empty()) {
        throw std::invalid_argument("输入向量不能为空");
    }
    double len = length(v);
    // 零向量不能做单位化，避免除零
    if (std::fabs(len) < 1e-9) {
        throw std::invalid_argument("零向量无法进行单位化操作");
    }
    return scale(v, 1.0 / len);
}

double angleBetween(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        throw std::invalid_argument("参与运算的向量维度必须一致");
    }
    if (a.empty()) {
        throw std::invalid_argument("输入向量不能为空");
    }
    double dot_product = dot(a, b);
    double len_a = length(a);
    double len_b = length(b);
    // 任意一个向量是零向量，无法计算夹角
    if (std::fabs(len_a) < 1e-9 || std::fabs(len_b) < 1e-9) {
        throw std::invalid_argument("零向量无法计算夹角");
    }
    // 数值裁剪避免浮点误差导致acos输入超出[-1,1]范围
    double cos_theta = dot_product / (len_a * len_b);
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    return std::acos(cos_theta);
}

} // namespace rm
