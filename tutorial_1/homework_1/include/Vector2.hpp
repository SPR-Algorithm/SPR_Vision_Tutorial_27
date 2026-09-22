#pragma once
#include<vector>
namespace rm{
    using Vector = std::vector<double>;
    using Matrix = std::vector<std::vector<double>>; 
    
    double length(const Vector& l);
    double distance(const Vector& a,const Vector& b);
    double dot(const Vector& a,const Vector& b);
    Vector scale(const Vector& l,double k);
    Vector normalize(const Vector& l);
    double angleBetween(const Vector& a,const Vector& b);
}