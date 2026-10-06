#include"camera.hpp"
std::string UsbCamera::name()const
{
     return "UsbCamera-"+id;
}
void UsbCamera::open()
{
    status=true;
    std::cerr<<id<<" 成功开启"<<std::endl;
}
void UsbCamera::close()
{
 status=false;
 std::cerr<<id<<" 成功关闭"<<std::endl;
}
bool UsbCamera::read(Frame&m)
{
    if(status)
    {
       m.cols=100;
       m.rows=100;
       m.color.assign(m.cols*m.rows*3,8*frame_);
       frame_++;
    }
    return status;
}
void IndustrialCamera::open()
{
    status=true;
    std::cerr<<id<<" 成功开启"<<std::endl;
}
void IndustrialCamera::close()
{
     status=false;
    std::cerr<<id<<" 失败开启"<<std::endl;
}
std::string IndustrialCamera::name()const
{
    return "IndustrialCamera-"+id;
}
bool IndustrialCamera::read(Frame&m)
{
    if(status)
    {
        m.rows=800;
        m.cols=800;
        m.color.assign(m.rows*m.cols*3,frame_*8);
        frame_++;
    }
    return status;
}
std::unique_ptr<Camera>makeCamera(std::string type,std::string id)
{
    if(type=="Usb")
    {
        std::cerr<<"Usbcamera-"<<id<<"已创建成功"<<std::endl;
        return std::make_unique<UsbCamera>(id);
    }
    else if(type=="Industrial")
    {
       std::cerr<<"IndustrialCamera-"<<id<<"创建成功"<<std::endl;
       return std::make_unique<IndustrialCamera>(id);
    }
    else
    {
        std::cerr<<"无法识别该类型"<<"相机创建失败"<<std::endl;
        throw std::invalid_argument("error");
    }
}