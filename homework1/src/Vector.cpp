#include "Vec2.hpp"
#include<cmath>
namespace rm{
    double length(const Vector& l){
        double sum=0.0;
        for(int i=0;i<l.size();i++)
            sum+=l[i]*l[i];
        return std::sqrt(sum);
    }

    double distance(const Vector& a,const Vector& b){
        if(length(a)!=length(b))
        return 0;

        double sum=0.0;
        for(int i=0;i<a.size();i++)
            sum+=((a[i]-b[i])*(a[i]-b[i]))
        return std::sqrt(sum);
    }  

    double dot(const Vector& a,const Vector& b){
        if(length(a)!=length(b))
        return 0;

        double sum=0.0;
        for(int i=0;i<a.size();i++)
        sum+=a[i]*b[i];
        return sum;
    }
 
    Vector scale(const Vector& l,double k){
        Vector m;
        for(int i=0;i<l.size();i++)
        m.push_back(l[i]*k);
        return m;
    }

    Vector normalize(const Vector& l){
        if(length(l)==0)
        return l;

        Vector n;
        for(int i=0;i<l.size();i++)
        n.push_back(l[i]/lengh(l));
        return n;
    }

    double angleBetween(const Vector& a,const Vector& b){
        if(length(a)==0||length(b)==0)
        return 0;
  
        if(length(a)!=length(b))
        return 0;
    
        double l1=length(a);
        double l2=length(b);
        double T=dot(a,b)
        double r=t/(l1*l2)
        return r;
    }
}