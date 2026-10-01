#include "vector.hpp"
#include <cmath>
#include <stdexcept>

namespace rm {

// 计算向量的模长（sqrt(x²+y²+...））
double length(const Vector& v) {
    double sum = 0.0;
    for (double x : v) {
        sum += x * x;
    }
    return std::sqrt(sum);
}

// 计算两个向量之间的欧氏距离
double distance(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        throw std::logic_error("两个向量维度不一致");
    }
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

// 计算两个向量的点乘
double dot(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        throw std::logic_error("两个向量维度不一致");
    }
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        sum += a[i] * b[i];
    }
    return sum;
}

// 向量乘以常数 k
Vector scale(const Vector& v, double k) {
    Vector result(v.size());
    for (size_t i = 0; i < v.size(); i++) {
        result[i] = v[i] * k;
    }
    return result;
}

// 单位化：变成长度为 1 的向量
Vector normalize(const Vector& v) {
    double len = length(v);
    if (len < 1e-9) {
        throw std::logic_error("零向量不能单位化");
    }
    return scale(v, 1.0 / len);
}

// 计算两个向量的夹角（返回弧度）
double angleBetween(const Vector& a, const Vector& b) {
    double lenA = length(a);
    double lenB = length(b);
    if (lenA < 1e-9 || lenB < 1e-9) {
        throw std::logic_error("零向量不能计算夹角");
    }
    double cosTheta = dot(a, b) / (lenA * lenB);
    if (cosTheta > 1.0) cosTheta = 1.0;
    if (cosTheta < -1.0) cosTheta = -1.0;
    return std::acos(cosTheta);
}

} 
