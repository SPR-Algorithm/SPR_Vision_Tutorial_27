#include"camera.hpp"
int main()
{
    std::unique_ptr<Camera>C1=makeCamera("Usb","001");
    std::cerr<<C1->name()<<std::endl;
    return 0;
}