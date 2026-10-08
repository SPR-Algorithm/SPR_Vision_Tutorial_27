#include "vector.hpp"
#include <cmath>

namespace rm{

double length(const Vector& v){
    if (v.empty()) return 0.0;
    
    double sum = 0.0;
    int m = v.size();
    for(int i=0;i<m;i++){
        if (std::isnan(v[i]) || std::isinf(v[i])) return 0.0;
        sum += v[i] * v[i];
    }
    if (std::isnan(sum) || std::isinf(sum)) return 0.0;
    return std::sqrt(sum);
}
double distance(const Vector& a,const Vector& b){
    if (a.size()!=b.size()) return 0.0;
    if (a.empty() || b.empty()) return 0.0;
    int n = a.size();
    for(int i=0;i<n;i++){ 
    if (std::isnan(a[i]) || std::isinf(b[i]))
    return 0.0;
    }
    for(int i=0;i<n;i++){
        if (std::isnan(b[i]) || std::isinf(b[i]))
        return 0.0;
    }
    double sum1 = 0.0;
    for(int i=0;i<n;i++){
        sum1 += (a[i]-b[i])*(a[i]-b[i]);
    }
    return std::sqrt(sum1);
}
double dot(const Vector& a, const Vector& b){
    if (a.size() != b.size()) return 0.0;
    int m1=a.size();
    double sum2=0.0;
    for (int i=0;i<m1;i++){
        sum2+=a[i]*b[i];
    }
    return sum2;
}
Vector scale(const Vector& v, double k){
    if (v.empty()) return {};
    int m2=v.size();
    Vector result;
    result.reserve(m2);
    for(int i=0;i<m2;i++){
        if (std::isnan(v[i]) || std::isinf(v[i]))
        return {};
        result.push_back(v[i]*k);
    }
    return result;
}
Vector normalize(const Vector& v){
    if (v.empty()) return {};
    double len=rm::length(v);
    if(len==0.0) return {};
    int m3=v.size();
    Vector result1;
    result1.reserve(m3);
    for(int i=0;i<m3;i++){
        result1.push_back(v[i]/len);
    }
    return result1;
}
double angleBetween(const Vector& a, const Vector& b){
    if (a.size() != b.size()) return 0.0;
    double lena=rm::length(a);
    double lenb=rm::length(b);
    if (lena==0 || lenb==0) return 0.0;
    double dotan=rm::dot(a,b);
    double result2=dotan/(lena*lenb);
    return result2;
}
}
