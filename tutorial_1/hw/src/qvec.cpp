#include "qvec.hpp"
#include <cmath>

namespace rm {

    double length(const Vector& v) {
        return std::sqrt((v.x*v.x) + (v.y*v.y));
    }

    double distance(const Vector& a, const Vector& b) {
        return length(Vector{a.x - b.x, a.y-b.y});
    }

    double dot(const Vector& a, const Vector& b) {
        return ((a.x * b.x) + (a.y * b.y));        
    }

    Vector scale(const Vector& v, double k) {
        return Vector{v.x*k, v.y*k};
    }

    Vector normalize(const Vector& v) {
        double l = length(v);
        return Vector{v.x/l, v.y/l};
    }

    double angleBetween(const Vector& a, const Vector& b) {
        return std::acos(dot(a, b)/(length(a)*length(b)))  * 180.0 / M_PI;
    }



}