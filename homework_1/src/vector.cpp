#include "vector.hpp"
#include <cmath>
#include <stdexcept>
namespace rm{
    double length(const Vector&v) {
        double sum = 0.0;
        for (double x : v)sum += x * x;
        return std::sqrt(sum);
    }
    double distance(const Vector& a,const Vector&b){
        if (a.size() != b.size())
            throw std::logic_error("dimension mismatch");
        Vector diff(a.size());
        for (size_t i = 0; i < a.size(); ++i)diff[i] = a[i] - b[i];
        return length(diff);
    }
    double dot_product(const Vector&a,const Vector&b){
        if (a.size() != b.size())
            throw std::logic_error("dimension mismatch");
        double sum = 0.0;
        for (size_t i = 0; i < a.size(); ++i)sum += a[i] * b[i];
        return sum;
    }
    Vector scale(const Vector&v,double k){
        Vector res(v.size());
        for (size_t i = 0; i < v.size(); ++i)res[i] = v[i] * k;
        return res;
    }
    Vector normalize(const Vector&v) {
        double len = length(v);
        if (len == 0.0)
            throw std::logic_error("cannot normalize zero vector");
        return scale(v, 1.0 / len);
    }
    double angleBetween(const Vector&a,const Vector&b){
        double denom = length(a) * length(b);
        if (denom == 0.0)
            throw std::logic_error("zero vector in angle calculation");
        double cos_theta = dot_product(a, b) / denom;
        if (cos_theta > 1.0)cos_theta = 1.0;
        if (cos_theta < -1.0)cos_theta = -1.0;
        return std::acos(cos_theta);
    }
}