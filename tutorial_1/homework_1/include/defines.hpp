#pragma once
#include<string>
#include <iostream>
#include<vector>
#include<cmath>

namespace rm
{
    using Vector = std::vector<double>;
    using  Matrix = std::vector<std::vector<double>>;
    double length(const Vector&);
    extern double distance(const Vector&, const Vector&);
    extern double dot(const Vector&, const Vector&);
    extern Vector scale(const Vector&, double k);
    extern Vector normalize(const Vector&);
    extern double angleBetween(const Vector&, const Vector&);
    extern Matrix zero(int, int);
    extern Matrix ones(int, int);
    extern Matrix random(int, int, double, double);
    extern void show(const Matrix&);
    extern Matrix multiply(const Matrix&,double);
    extern Matrix multiply(const Matrix&, const Matrix&);
    extern Matrix sum(const Matrix&, double c);
}