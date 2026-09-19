#pragma once
namespace rm{
    struct Vec2 {
        double x = 0.0;
        double y = 0.0;
    };
    double distance(const Vec2 &a,const Vec2 &b);
    double length(const Vec2 &v);
    double dot(const Vec2 &a,const Vec2 &b);
    Vec2 scale(const Vec2 &a,double k);
}