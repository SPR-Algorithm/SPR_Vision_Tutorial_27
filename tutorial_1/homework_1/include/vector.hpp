#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace rm {

using Vector = std::vector<double>;


class NonFinite : public std::runtime_error {
public:
    explicit NonFinite(const std::string& what) : std::runtime_error(what) {}
};

double length(const Vector& v);

double distance(const Vector& a, const Vector& b);

double dot(const Vector& a, const Vector& b);

Vector scale(const Vector& v, double k);

Vector normalize(const Vector& v);

double angleBetween(const Vector& a, const Vector& b);

}  // namespace rm
