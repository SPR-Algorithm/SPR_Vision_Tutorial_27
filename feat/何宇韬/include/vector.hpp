#pragma once
#include <vector>
namespace rm{
    using Vector = std::vector<double>;
    using Matrix = std::vector<std::vector<double>>;
    double length(const Vector& v);
    double distance(const Vector& a, const Vector& b);
    double dot(const Vector& a, const Vector& b);
    Vector scale(const Vector& v, double k);
    Vector normalize(const Vector& v);
    double angleBetween(const Vector& a, const Vector& b);

}