#include <iostream>
#include "qvec.hpp"

int main(){
    const rm::Vector a{6.0, 8.0};
    const rm::Vector b{3.0, 4.0};
    
    std::cout << "|a| = " << rm::length(a);
    std::cout << "\n|b| = " << rm::length(b);
    std::cout << "\na*b = " << rm::dot(a, b) << "\n";
    

    return 0;

}