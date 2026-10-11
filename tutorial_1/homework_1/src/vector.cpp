#include "vector.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace rm {

namespace {

constexpr double kZeroEps = 1e-9;

void checkSameSize(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
        throw std::logic_error("Vector dimensions do not match");
    }
}

double sumOfSquares(const Vector& v) {
    double sum = 0.0;
    for (double x : v) {
        sum += x * x;
    }
    return sum;
}

} 

double length(const Vector& v) {
    return std::sqrt(sumOfSquares(v));
}

double distance(const Vector& a, const Vector& b) {
    checkSameSize(a, b);
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

double dot(const Vector& a, const Vector& b) {
    checkSameSize(a, b);
    double result = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        result += a[i] * b[i];
    }
    return result;
}

Vector scale(const Vector& v, double k) {
    Vector result(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        result[i] = v[i] * k;
    }
    return result;
}

Vector normalize(const Vector& v) {
    const double len = length(v);
    if (len < kZeroEps) {
        return Vector(v.size(), 0.0);
    }
    return scale(v, 1.0 / len);
}

double angleBetween(const Vector& a, const Vector& b) {
    checkSameSize(a, b);

    const double lenA = length(a);
    const double lenB = length(b);
    if (lenA < kZeroEps || lenB < kZeroEps) {
        return 0.0;
    }

    double cosTheta = dot(a, b) / (lenA * lenB);
    if (cosTheta > 1.0) cosTheta = 1.0;
    if (cosTheta < -1.0) cosTheta = -1.0;

    return std::acos(cosTheta);
}

} 