#include <iostream>
#include "fax.hpp"
int main(){
    const rm::Vec2 a{3.0,4.0};
    const rm::Vec2 b{0.0,0.0};
    std::cout << "|a|    =     " <<rm::length(a) << "\n";
    std::cout << "dist(a,b)=   " << rm::distance(a, b)<<"\n";
    std::cout << "a*b    =     " << rm::dot(a,b)<<"\n";
    const rm::Vec2 c = rm::scale(a,2.0);
    std::cout << "2a     =    (" << c.x << ","<<c.y << ")\n";
    return 0;

}
