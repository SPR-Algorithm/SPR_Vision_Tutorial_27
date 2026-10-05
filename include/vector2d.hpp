#pragma once
#include <vector>
#include <stdexcept>

class Vector2d {
public:
    double x, y;
    Vector2d(double x_, double y_) : x(x_), y(y_) {}
    Vector2d operator+(const Vector2d& other) const {
        return Vector2d(x + other.x, y + other.y);
    }
};

namespace rm {
using Vector = std::vector<double>;
using Matrix = std::vector<std::vector<double>>;

double length(const Vector& v);
double distance(const Vector& a, const Vector& b);
double dot(const Vector& a, const Vector& b);
Vector scale(const Vector& v, double k);
Vector normalize(const Vector& v);
double angleBetween(const Vector& a, const Vector& b);
}