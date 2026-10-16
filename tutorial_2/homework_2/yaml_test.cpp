#include <iostream>
#include "tools/yaml.hpp"   
using namespace std;

int main() {
   
    YAML::Node cfg = tools::load("configs/camera.yaml");

   
    string name    = tools::read<string>(cfg, "camera_name");
    int exposure   = tools::read<int>(cfg, "exposure_ms");
    double gain    = tools::read<double>(cfg, "gain");
    int flip       = tools::read<int>(cfg, "flip_code");

  
    cout << "相机名: " << name << endl;
    cout << "曝光: " << exposure << " ms" << endl;
    cout << "增益: " << gain << endl;
    cout << "翻转码: " << flip << endl;

    return 0;
}
