#include "vector.hpp"
#include <cmath>

namespace rm {

double length(const Vector& v) {
    double sum = 0.0;
    for (double x : v) sum += x * x;
    return std::sqrt(sum);
}

double distance(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) return 0.0;
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

double dot(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) return 0.0;
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) sum += a[i] * b[i];
    return sum;
}

Vector scale(const Vector& v, double k) {
    Vector res;
    for (double x : v) res.push_back(x * k);
    return res;
}

Vector normalize(const Vector& v) {
    double len = length(v);
    if (len == 0.0) {
        return Vector(v.size(), 0.0);
    }
    return scale(v, 1.0 / len);
}

double angleBetween(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) return 0.0;
    double len_a = length(a);
    double len_b = length(b);
    if (len_a == 0.0 || len_b == 0.0) return 0.0;
    double cos_theta = dot(a, b) / (len_a * len_b);
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    return std::acos(cos_theta);
}

}  //namespace rm



