#include "vector.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace rm {
namespace {

// 模长小于该阈值即视为零向量（与作业测试里的 1e-9 容差一致）。
constexpr double kZeroTolerance = 1e-9;

// 所有元素必须为有限值。
void requireFinite(const Vector& v, const char* fn) {
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (!std::isfinite(v[i])) {
            throw std::domain_error(std::string(fn) +
                                    ": vector element at index " + std::to_string(i) +
                                    " is not finite");
        }
    }
}

// 参与二元运算的两个向量维度必须一致。
void requireSameDimension(const Vector& a, const Vector& b, const char* fn) {
    if (a.size() != b.size()) {
        throw std::invalid_argument(std::string(fn) + ": dimension mismatch (" +
                                    std::to_string(a.size()) + " vs " +
                                    std::to_string(b.size()) + ")");
    }
}

// 以 long double 计算的 2-范数，不做 double 范围检查（供内部复用）。
long double normLong(const Vector& v) {
    long double sum = 0.0L;
    for (double x : v) {
        const long double t = static_cast<long double>(x);
        sum += t * t;
    }
    return std::sqrt(sum);
}

// 把 long double 结果安全地收窄为有限的 double，溢出即抛异常。
double toFiniteDouble(long double value, const char* fn) {
    const long double limit =
        static_cast<long double>(std::numeric_limits<double>::max());
    if (!std::isfinite(value) || value > limit || value < -limit) {
        throw std::domain_error(std::string(fn) +
                                ": result is not representable as a finite double");
    }
    return static_cast<double>(value);
}

}  // namespace

double length(const Vector& v) {
    requireFinite(v, "length");
    return toFiniteDouble(normLong(v), "length");
}

double distance(const Vector& a, const Vector& b) {
    requireSameDimension(a, b, "distance");
    requireFinite(a, "distance");
    requireFinite(b, "distance");

    long double sum = 0.0L;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const long double d =
            static_cast<long double>(a[i]) - static_cast<long double>(b[i]);
        sum += d * d;
    }
    return toFiniteDouble(std::sqrt(sum), "distance");
}

double dot(const Vector& a, const Vector& b) {
    requireSameDimension(a, b, "dot");
    requireFinite(a, "dot");
    requireFinite(b, "dot");

    long double sum = 0.0L;
    for (std::size_t i = 0; i < a.size(); ++i) {
        sum += static_cast<long double>(a[i]) * static_cast<long double>(b[i]);
    }
    return toFiniteDouble(sum, "dot");
}

Vector scale(const Vector& v, double k) {
    requireFinite(v, "scale");
    if (!std::isfinite(k)) {
        throw std::domain_error("scale: factor is not finite");
    }

    Vector out;
    out.reserve(v.size());
    for (double x : v) {
        out.push_back(toFiniteDouble(static_cast<long double>(x) * k, "scale"));
    }
    return out;
}

Vector normalize(const Vector& v) {
    requireFinite(v, "normalize");

    const long double len = normLong(v);

    // 零向量 / 近零向量：不做除法，返回同维度的全 0 向量（空向量仍为空）。
    if (len < static_cast<long double>(kZeroTolerance)) {
        return Vector(v.size(), 0.0);
    }

    Vector out;
    out.reserve(v.size());
    for (double x : v) {
        // |x| <= ||v||，故比值天然落在 [-1, 1]，不会溢出。
        out.push_back(static_cast<double>(static_cast<long double>(x) / len));
    }
    return out;
}

double angleBetween(const Vector& a, const Vector& b) {
    requireSameDimension(a, b, "angleBetween");
    requireFinite(a, "angleBetween");
    requireFinite(b, "angleBetween");

    const long double la = normLong(a);
    const long double lb = normLong(b);

    // 任一方为零向量 / 近零向量：按约定夹角为 0，不产生 NaN。
    if (la < static_cast<long double>(kZeroTolerance) ||
        lb < static_cast<long double>(kZeroTolerance)) {
        return 0.0;
    }

    // 先各自单位化再点乘，避免 ||a|| * ||b|| 的中间溢出。
    long double cosTheta = 0.0L;
    for (std::size_t i = 0; i < a.size(); ++i) {
        cosTheta += (static_cast<long double>(a[i]) / la) *
                    (static_cast<long double>(b[i]) / lb);
    }

    // 浮点误差可能让 cos 略微越界，夹紧后 acos 不会产生 NaN。
    double cosValue = static_cast<double>(cosTheta);
    if (cosValue > 1.0) cosValue = 1.0;
    if (cosValue < -1.0) cosValue = -1.0;

    return std::acos(cosValue);
}

}  // namespace rm
