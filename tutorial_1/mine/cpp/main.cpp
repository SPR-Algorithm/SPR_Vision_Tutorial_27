#include"defines.hpp"
int main()
{
    rm::Vector2 A1(1,2);
    rm::Vector2 A2(3,4);
    rm::Vector2 A3=A1+A2;
    std::cout<<length(A3)<<std::endl;
   return 0;
}

