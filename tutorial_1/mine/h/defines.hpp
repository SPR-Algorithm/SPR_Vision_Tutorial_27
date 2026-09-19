#include<iostream>
#include<cmath>
namespace rm
{
 struct Vector2
 {
         double x;double y;
         Vector2(double x_,double y_):x(x_),y(y_){}
         Vector2&operator=(const Vector2 o){x=o.x;y=o.y;return*this;}
         Vector2 operator+(const Vector2 o){return Vector2(x+o.x,y+o.y);}
         Vector2 operator*(double a){return Vector2(a*x,a*y);}
         Vector2 operator-(const Vector2 o){return Vector2(x-o.x,y-o.y);}
        double operator*(Vector2 o){return x*o.x+y*o.y;}
  };
}
double length(rm::Vector2);
