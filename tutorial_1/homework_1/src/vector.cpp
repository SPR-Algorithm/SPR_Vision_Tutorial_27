#include "../include/vector.hpp"
#include <cmath>
#include <cstddef>
#include <stdexcept>
namespace rm{
    double length (const Vector& v){
        double sum=0.0;
        for (double x: v){
            sum+= x*x;
        }
        return std::sqrt(sum);
    }
    double distance (const Vector& a, const Vector& b){
        if (a.size()!=b.size()){
            throw std::invalid_argument("distance:vector dimensions mismatch");
        }
        double sum=0.0;
        for (size_t i=0;i<a.size();++i){
            double diff = a[i]-b[i];
            sum+= diff*diff;

        }
        return std::sqrt(sum);
    }
    double dot (const Vector& a, const Vector &b){
        if (a.size()!=b.size()){
            throw std::invalid_argument("dot: vector dimensions mismatch");
        }
        double sum=0.0;
        for (size_t i=0;i<a.size();++i){
            sum+=a[i]*b[i];
        }
        return sum;
    }
    Vector scale (const Vector& v, double k){
        Vector res (v.size());
        for (size_t i=0; i<v.size(); ++i){
            res[i]=v[i]*k;
        }
        return res;
    } 
    Vector normalize (const Vector &v){
        double len = length (v);
        if (len==0.0){
            throw std::invalid_argument ("normalize: zero vector cannot be normalized");
        }
        return scale (v, 1.0/len);
    }
    double angleBetween (const Vector& a, const Vector& b){
        if (a.size()!=b.size()){
            throw std::invalid_argument("angelBetween: vector dimensions mismatch");
        }
        double lena= length (a);
        double lenb= length(b);
        if (lena==0.0||lenb==0.0){
            throw std::invalid_argument("angelBetween: zero vector has no angel");
        }
        double cos_theta = dot(a,b)/(lena*lenb);
        if (cos_theta>1.0) cos_theta=1.0;
        if(cos_theta<-1.0) cos_theta=-1.0;
        return std::acos(cos_theta);

    } 

}

