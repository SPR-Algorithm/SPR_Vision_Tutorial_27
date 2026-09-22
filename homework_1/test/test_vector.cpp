#include"vector.hpp"
#include<iostream>
int main(){
    rm::Vector a={1.0,2.0,3.0};
    rm::Vector b={4.0,5.0,6.0};
    std::cout << "length(a) = " << rm::length(a) << "\n";
    std::cout << "distance(a,b) = " << rm::distance(a, b) << "\n";
    std::cout << "dot(a,b) = " << rm::dot_product(a, b) << "\n";
    return 0;
}