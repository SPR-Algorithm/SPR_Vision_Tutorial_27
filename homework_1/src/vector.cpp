#include "vector.hpp"
#include<cmath>
#include<cstddef>
#include<stdexcept>
namespace {

void ensureNotEmpty(const rm::Vector& v) {
    if (v.empty()) {
        throw std::logic_error("rm::vector: input vector must not be empty");
    }
}

void ensureSameSize(const rm::Vector& a, const rm::Vector& b) {
    ensureNotEmpty(a);
    ensureNotEmpty(b);
    if (a.size() != b.size()) {
        throw std::logic_error("rm::vector: dimension mismatch");
    }
}
}  
namespace rm{

double length(const Vector& v) {
    ensureNotEmpty(v);
    double sum = 0.0;
    for (double x : v) {
        sum += x * x;
    }
   
    if (!std::isfinite(sum)) {
        throw std::logic_error("rm::length: result is not finite");
    }
    return std::sqrt(sum);
}

double distance(const Vector& a, const Vector& b) {
    ensureSameSize(a, b);

    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i];
        sum += d * d;
    }

    if (!std::isfinite(sum)) {
        throw std::logic_error("rm::distance: result is not finite");
    }
    return std::sqrt(sum);
}


double dot(const Vector& a, const Vector& b) {
    ensureSameSize(a, b);

    double s = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        s += a[i] * b[i];
    }

    if (!std::isfinite(s)) {
        throw std::logic_error("rm::dot: result is not finite");
    }
    return s;
}

Vector scale(const Vector& v, double k) {
    ensureNotEmpty(v);
    if (!std::isfinite(k)) {
        throw std::logic_error("rm::scale: scale factor must be finite");
    }

    Vector result;
    result.reserve(v.size());        
    for (double x : v) {
        const double y = x * k;
        if (!std::isfinite(y)) {
            throw std::logic_error("rm::scale: result is not finite");
        }
        result.push_back(y);
    }
    return result;
}

Vector normalize(const Vector& v) {
    ensureNotEmpty(v);

    const double len = length(v);
    if (len == 0.0) {                  
        throw std::logic_error("rm::normalize: cannot normalize a zero vector");
    }

    Vector result;
    result.reserve(v.size());
    for (double x : v) {
        result.push_back(x / len);
    }
    return result;
}

double angleBetween(const Vector& a, const Vector& b) {
    ensureSameSize(a, b);

    const double lenA = length(a);
    const double lenB = length(b);
    if (lenA == 0.0 || lenB == 0.0) {
        throw std::logic_error("rm::angleBetween: zero vector has no direction");
    }

    double cosine = dot(a, b) / lenA / lenB;

    if (cosine > 1.0) {
        cosine = 1.0;
    }
    if (cosine < -1.0) {
        cosine = -1.0;
    }

    return std::acos(cosine);
}

}