#include "vector2d.hpp"
#include <cmath>

namespace rm {

double length(const Vector& v) {
    double sum_sq = 0.0;
    for (auto val : v) {
        sum_sq += val * val;
    }
    return std::sqrt(sum_sq);
}

double distance(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        throw std::logic_error("Vector dimension mismatch");
    }
    Vector diff(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        diff[i] = a[i] - b[i];
    }
    return length(diff);
}

double dot(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        throw std::logic_error("Vector dimension mismatch");
    }
    double res = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        res += a[i] * b[i];
    }
    return res;
}

Vector scale(const Vector& v, double k) {
    Vector res(v.size());
    for (size_t i = 0; i < v.size(); ++i) {
        res[i] = v[i] * k;
    }
    return res;
}

Vector normalize(const Vector& v) {
    double len = length(v);
    if (std::fabs(len) < 1e-12) {
        throw std::logic_error("Zero vector cannot be normalized");
    }
    return scale(v, 1.0 / len);
}

double angleBetween(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        throw std::logic_error("Vector dimension mismatch");
    }
    double dot_val = dot(a, b);
    double len_a = length(a);
    double len_b = length(b);
    if (std::fabs(len_a) < 1e-12 || std::fabs(len_b) < 1e-12) {
        throw std::logic_error("Zero vector has no defined angle");
    }
    double cos_ang = dot_val / (len_a * len_b);
    // 浮点误差保护，限制值域 [-1,1]
    cos_ang = std::max(-1.0, std::min(1.0, cos_ang));
    return std::acos(cos_ang);
}

}