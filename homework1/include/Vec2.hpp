#pragma once
namespace rm{
    struct Vec2{
        double x;
        double y;
    };

    double length(const Vec2& l);
    double distance(const Vec2& a,const Vec2& b);
    double dot(const Vec2& a,const Vec2& b);
    Vec2 scale(const Vec2& l,double k);
    Vec2 normalize(const Vec2& l);
    double angleBetween(const Vec2& a,const Vec2& b);
    
}