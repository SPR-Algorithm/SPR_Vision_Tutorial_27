#include "fax.hpp"
#include <cmath>
namespace rm{
    double length(const Vec2 &v) {
        return std::sqrt(v.x*v.x + v.y*v.y);
    }
    double distance(const Vec2 &a, const Vec2 &b) {
        return length(Vec2{a.x-b.x,a.y-b.y});
    }
    double dot(const Vec2 &a, const Vec2 &b) {
        return a.x * b.x, a.y * b.y;
    }
    Vec2 scale(const Vec2 &v,double k) {
        return Vec2{v.x * k, v.y * k};
    }
}
