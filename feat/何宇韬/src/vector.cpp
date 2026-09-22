#include "vector.hpp"
#include <cmath>
#include <iostream>
namespace rm{
    double length(const Vector& v){
        double vector_length_squire = 0.0;
        for (size_t i = 0;i < v.size();i++){
            vector_length_squire = vector_length_squire + v[i]*v[i];

        }
        return std::sqrt(vector_length_squire);
        
    }
    double distance(const Vector& a, const Vector& b){
        if (a.size() == b.size()){
            Vector result(a.size());
            for (size_t i = 0;i < a.size();i++){
                result[i] = a[i] - b[i];
            }
            return length(result);
        }else{
            throw std::logic_error("向量维度不匹配");
            return 0;
        }
        
    }
    double dot(const Vector& a, const Vector& b){
        if (a.size() == b.size()){
            double dot_result = 0.0; 
        for (size_t i = 0;i <a.size();i++){
            dot_result = dot_result + a[i]*b[i];
        }
            return dot_result;
        }
        else{
            throw std::logic_error("向量维度不匹配");
            return 0;
        }
    }
    Vector scale(const Vector& v, double k){
        Vector scale_result(v.size());
        for (size_t i = 0;i<v.size();i++){
            scale_result[i] = v[i]*k;
        }
    
        return scale_result;
    }
    Vector normalize(const Vector& v) {
        if (length(v) <= 1e-12){
            Vector zero_vector(v.size());
            for (size_t i =0;i<v.size();i++){
                zero_vector[i]=0;
                
            }
            return zero_vector;
        }else{
            Vector normalize_result(v.size());
            for (size_t i = 0;i<v.size();i++){
                normalize_result[i] = v[i]/length(v);
            }
            return normalize_result;
        }
        
    }
    double angleBetween(const Vector& a, const Vector& b){
        if (a.size() == b.size()){
            if (dot(a,b) == 0){
                if (length(a)==0||length(b)==0){
                    return 0;
                }else{
                    return 3.1415926535897932/2.0;

                }
                
            }else{
                if(dot(a,b)/(length(a)*length(b))>1){
                    return 0;
                }else if (dot(a,b)/(length(a)*length(b))<-1){
                    return 3.1415926535897932;
                    
                }else{
                    return acos(dot(a,b)/(length(a)*length(b)));

                }
                
                
            }
        }else{
            throw std::logic_error("向量维度不匹配");
            return 0;
        }

    }

}
