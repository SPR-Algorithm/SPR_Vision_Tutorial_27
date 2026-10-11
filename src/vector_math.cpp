#include "vector_math.h"
#include <cmath>      
#include <stdexcept>  
#include <algorithm>  

namespace rm {


static void checkSameDimension(const Vector& a, const Vector& b) {
    if (a.size() != b.size()) {
       
        throw std::invalid_argument("Vectors must have the same dimension.");
    }
}


double length(const Vector& v) {
    double sum = 0.0;
    for (double x : v) {  
        sum += x * x;     
    }
    return std::sqrt(sum); 
}


double distance(const Vector& a, const Vector& b) {
    checkSameDimension(a, b); 
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        double diff = a[i] - b[i]; 
        sum += diff * diff;       
    }
    return std::sqrt(sum);      
}


double dot(const Vector& a, const Vector& b) {
    checkSameDimension(a, b); 
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        sum += a[i] * b[i];  
    }
    return sum; 
}


Vector scale(const Vector& v, double k) {
    Vector result(v.size()); 
    for (size_t i = 0; i < v.size(); ++i) {
        result[i] = v[i] * k; 
    }
    return result; 
}


Vector normalize(const Vector& v) {
    double len = length(v); 
    if (len < 1e-9) { 
        return v;     
    return scale(v, 1.0 / len); 
}

double angleBetween(const Vector& a, const Vector& b) {
    checkSameDimension(a, b); 
    double len_a = length(a);
    double len_b = length(b);
    
    
    if (len_a < 1e-9 || len_b < 1e-9) {
        throw std::runtime_error("Cannot compute angle with zero vector.");
    }
    
    
    double cos_theta = dot(a, b) / (len_a * len_b);
    
    
    cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    
   
    return std::acos(cos_theta);
}

    }