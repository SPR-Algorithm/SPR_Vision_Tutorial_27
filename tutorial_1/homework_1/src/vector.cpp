#include "vector.hpp"
#include <cmath>
#include <string>
#include <algorithm>

namespace rm {
namespace {

double dropNegativeZero(double x) {
    return (x == 0.0) ? 0.0 : x;
}

double requireFinite(double x, const char* what) {
    if (!std::isfinite(x)) {
        throw NonFinite(std::string(what) + ": 结果不是有限值");
    }
    return x;
}

void requireFiniteElements(const Vector& v, const char* what) {
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (!std::isfinite(v[i])) {
            throw NonFinite(std::string(what) + ": 第 " + std::to_string(i) +
                            " 个分量不是有限值");
        }
    }
}

void requireSameSize(const Vector& a, const Vector& b, const char* what) {
    if (a.size() != b.size()) {
        throw std::invalid_argument(std::string(what) + ": 维度不一致（" +
                                    std::to_string(a.size()) + " vs " +
                                    std::to_string(b.size()) + "）");
    }
}

void requireFiniteScalar(double k, const char* what) {
    if (!std::isfinite(k)) {
        throw NonFinite(std::string(what) + ": 缩放系数不是有限值");
    }
}

}  // namespace

double length(const Vector& v) {
    requireFiniteElements(v, "length");

    double scale = 0.0;
    for (double x : v) {
        scale = std::max(scale, std::abs(x));
    }
    if (scale == 0.0) {
        return 0.0;  
    }

    double sum = 0.0;
    for (double x : v) {
        const double q = x / scale;
        sum += q * q;
    }
    return requireFinite(scale * std::sqrt(sum), "length");
}

double distance(const Vector& a, const Vector& b) {
    requireSameSize(a, b, "distance");
    requireFiniteElements(a, "distance");
    requireFiniteElements(b, "distance");

    Vector diff(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i];
        if (!std::isfinite(d)) {
            throw NonFinite("distance: 分量相减后溢出为无穷大");
        }
        diff[i] = d;
    }
    return length(diff);
}

double dot(const Vector& a, const Vector& b) {
    requireSameSize(a, b, "dot");
    requireFiniteElements(a, "dot");
    requireFiniteElements(b, "dot");

    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        sum += a[i] * b[i];
        if (!std::isfinite(sum)) {
            throw NonFinite("dot: 累加结果溢出为无穷大");
        }
    }
    return dropNegativeZero(sum);
}

Vector scale(const Vector& v, double k) {
    requireFiniteElements(v, "scale");
    requireFiniteScalar(k, "scale");

    Vector out(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        const double p = v[i] * k;
        if (!std::isfinite(p)) {
            throw NonFinite("scale: 第 " + std::to_string(i) + " 个分量溢出为无穷大");
        }
        out[i] = dropNegativeZero(p);
    }
    return out;
}

Vector normalize(const Vector& v) {
    requireFiniteElements(v, "normalize");

    const double len = length(v);
    if (len == 0.0) {
        throw std::invalid_argument("normalize: 零向量不能单位化（会除零）");
    }

    Vector out(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        out[i] = dropNegativeZero(v[i] / len);
    }
    return out;
}

double angleBetween(const Vector& a, const Vector& b) {
    requireSameSize(a, b, "angleBetween");
    requireFiniteElements(a, "angleBetween");
    requireFiniteElements(b, "angleBetween");

    const double lenA = length(a);
    const double lenB = length(b);
    if (lenA == 0.0 || lenB == 0.0) {
        throw std::invalid_argument("angleBetween: 零向量与任何向量的夹角无定义");
    }

    const double denom = lenA * lenB;
    if (!std::isfinite(denom) || denom <= 0.0) {
        throw NonFinite("angleBetween: 模长乘积不是有限正值，无法计算夹角");
    }

    double cosTheta = dot(a, b) / denom;
    if (!std::isfinite(cosTheta)) {
        throw NonFinite("angleBetween: 余弦值不是有限值");
    }
    cosTheta = std::clamp(cosTheta, -1.0, 1.0);
    return std::acos(cosTheta);
}

}  // namespace rm
