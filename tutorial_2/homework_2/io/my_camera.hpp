#include"hikrobot/include/MvCameraControl.h"
#include<opencv2/opencv.hpp>
#include<string>
#include<chrono>
#include<memory>
#include"../tools/logger.hpp"
#include"../tools/yaml.hpp"
namespace io
{
class CameraBase
{
public:
    virtual ~CameraBase()=default;
    virtual bool is_open()=0;
    virtual void read(cv::Mat&)=0;
};
class HikCamera:public CameraBase
{
public:
explicit HikCamera(double,double,std::string);
bool is_open()override;
~HikCamera()override{close();};
void read(cv::Mat&)override;
private:
double exposure;double gain;
void close();
void open();
void*handle=nullptr;
int vid=-1;
int pid=-1;
static MV_CC_DEVICE_INFO* pick(MV_CC_DEVICE_INFO_LIST&,int,int);
};
class UsbCamera:public CameraBase
{
public:
explicit UsbCamera(double,double,std::string);
explicit UsbCamera(double,double,unsigned int);
~UsbCamera()override=default;
bool is_open()override;
void read(cv::Mat&)override;
private:
void open();
cv::VideoCapture cat;
unsigned int device_id=0;
unsigned int pick_device();
double exposure=0;double gain=0;
int vid;int pid;
};
class Camera
{
public:
explicit Camera(std::string configs_path);
bool is_open();
void read(cv::Mat&);
private:
std::unique_ptr<CameraBase>Cam;
};
}