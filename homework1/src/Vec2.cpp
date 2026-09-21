#include "Vec2.hpp"
#include<cmath>
namespace rm{
    double length(const Vec2& l){
        return std::sqrt(l.x*l.x+l.y*l.y);
    }

    double distance(const Vec2& a,const Vec2& b){
        return std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    }  

    double dot(const Vec2& a,const Vec2& b){
        return a.x*b.x+a.y*b.y;
    }

    Vec2 scale(const Vec2& l,double k){
        return Vec2{l.x*k , l.y*k};
    }

    Vec2 normalize(const Vec2& l){
        return Vec2{l.x/l.x , l.y/l.y};
    }

    double angleBetween(const Vec2& a,const Vec2& b){
        double l1=std::sqrt(a.x*a.x+a.y*a.y);
        double l2=std::sqrt(b.x*b.x+b.y*b.y);
        return (a.x*b.x+a.y*b.y)/(l1*l2);
    }
}