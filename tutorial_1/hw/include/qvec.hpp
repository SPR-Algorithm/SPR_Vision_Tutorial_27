#pragma once 

namespace rm {
    struct Vector {
        double x;
        double y;
    };

    double length(const Vector& v);
    double distance(const Vector& a, const Vector& b);
    double dot(const Vector& a, const Vector& b);
    Vector scale(const Vector& a, double k);
    Vector normalize(const Vector& v);
    double angleBetween(const Vector& a, const Vector& b);





}