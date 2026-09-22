#include "Vector2.hpp"
#include<cmath>
#include <stdexcept>
namespace rm{
    double length(const Vector& v){
        double sum=0.0;
        for(size_t i=0;i<v.size();i++)
            sum+=v[i]*v[i];
        return std::sqrt(sum);
    }

    double distance(const Vector& a,const Vector& b){
        if(a.size()!=b.size()){
        throw std::logic_error("维度不匹配");
            }
    else{
        double sum=0.0;
        for(size_t i=0;i<a.size();i++)
            sum+=((a[i]-b[i])*(a[i]-b[i]));
        return std::sqrt(sum);
    }
}  

    double dot(const Vector& a,const Vector& b){
        if(a.size()!=b.size()){
            throw std::logic_error("维度不匹配");
        }

        double sum=0.0;
        for(size_t i=0;i<a.size();i++)
        sum+=a[i]*b[i];
        return sum;
    }
 
    Vector scale(const Vector& v,double k){
        Vector m;
        for(size_t i=0;i<v.size();i++)
        m.push_back(v[i]*k);
        return m;
    }

    Vector normalize(const Vector& v){
        if(length(v)==0){
        return v;}
        if(length(v)<1e-12)
        return Vector(1,0.0);
        Vector n;
        for(size_t i=0;i<v.size();i++)
        n.push_back(v[i]/length(v));
        return n;
    }

    double angleBetween(const Vector& a,const Vector& b){
        if(length(a)==0||length(b)==0){
            return 0;}
        if(a.size()!=b.size()){
        throw std::logic_error("维度不匹配");
           }
           double l1,l2;
        /*   for(size_t i=0;i<a.size();i++){
        l1+=a[i]*a[i];
        l2+=b[i]*b[i];}
        l1=std::sqrt(l1);
        l2=std::sqrt(l2); */
        l1=length(a);
        l2=length(b);
        double T=dot(a,b);
        double r=T/(l1*l2);
        if(r>1)r=1;
        if(r<-1)r=-1;
        return std::acos(r);
}
}