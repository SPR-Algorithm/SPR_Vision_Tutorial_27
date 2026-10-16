#include"io/my_camera.hpp"
#include"tools/yaml.hpp"
#include"hikrobot/include/MvCameraControl.h"
#include <unordered_map>
#include <stdexcept>
namespace io{
cv::Mat transfer(void*handle,MV_FRAME_OUT& raw)
{
    MV_CC_PIXEL_CONVERT_PARAM cvt_param{};
    cv::Mat img(raw.stFrameInfo.nHeight,raw.stFrameInfo.nWidth,CV_8UC3);
    cvt_param.nWidth =raw.stFrameInfo.nWidth;
    cvt_param.nHeight =raw.stFrameInfo.nHeight;
    cvt_param.pSrcData =raw.pBufAddr;
    cvt_param.nSrcDataLen =raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType =raw.stFrameInfo.enPixelType;
    cvt_param.pDstBuffer =img.data;
    cvt_param.nDstBufferSize =img.total()*img.elemSize();
    cvt_param.enDstPixelType =PixelType_Gvsp_BGR8_Packed;
    if (MV_CC_ConvertPixelType(handle,&cvt_param)!=MV_OK)  return{};

    return img;
}
HikCamera::HikCamera(double exposure_ms, double gain, const std::string& vid_pid): 
    exposure_ms_(exposure_ms), gain_(gain), vid_pid_(vid_pid){}

bool HikCamera::open()
{
    MV_CC_DEVICE_INFO_LIST device_list{};
    int ret=MV_CC_EnumDevices(MV_USB_DEVICE,&device_list);
    if(ret!=MV_OK)
        return false;
    if(device_list.nDeviceNum==0)
    {
        return false;
    }
    ret=MV_CC_CreateHandle(&handle_,device_list.pDeviceInfo[0]);
    if(ret!=MV_OK)   
        return false;
    ret=MV_CC_OpenDevice(handle_);
    if(ret!=MV_OK)
       return false;

    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);

    MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_ms_ * 1000.0);
    MV_CC_SetFloatValue(handle_, "Gain", gain_);

    ret=MV_CC_StartGrabbing(handle_);
    if(ret!=MV_OK)
       return false;
    return true;
}
cv::Mat HikCamera::read()
{
    MV_FRAME_OUT raw;
    int ret=MV_CC_GetImageBuffer(handle_,&raw,100);
    if (ret!=MV_OK)
    {
        return{};
    }   
    cv::Mat img=transfer(handle_,raw);
    MV_CC_FreeImageBuffer(handle_,&raw);
    return img;
}
void HikCamera::close()
{
    if(handle_==nullptr)    return;
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_=nullptr;
}

UsbCamera::UsbCamera(int index) : index_(index) {}
bool UsbCamera::open()
{
    return cap_.open(index_);
}
cv::Mat UsbCamera::read()
{
    cv::Mat img;
    if (!cap_.read(img))
    {
        return {};
    }
    return img;
}
void UsbCamera::close()
{
    cap_.release();
}
ReplayCamera::ReplayCamera(const std::string& path) : path_(path) {}

bool ReplayCamera::open()
{
    return cap_.open(path_);
}

cv::Mat ReplayCamera::read()
{
    cv::Mat img;
    if (!cap_.read(img))
    {
        return {};
    }
    return img;
}

void ReplayCamera::close()
{
    cap_.release();
}

Camera::Camera(const std::string& config_path)
{
    YAML::Node yaml = tools::load(config_path);

    const std::string camera_name=tools::read<std::string>(yaml,"camera_name");
    const double exposure_ms = tools::read<double>(yaml,"exposure_ms",3.0);
    const double gain=tools::read<double>(yaml,"gain",16.9);
    const std::string vid_pid = tools::read<std::string>(yaml,"vid_pid","");
    const int usb_index=tools::read<int>(yaml,"usb_index",0);
    const std::string replay_path = tools::read<std::string>(yaml,"replay_path","");
    flip_code_ = tools::read<int>(yaml,"flip_code",1);

    if(camera_name == "hikrobot"||camera_name == "hik")
    {
        camera_ = std::make_unique<HikCamera>(exposure_ms,gain,vid_pid);
    }
    else if(camera_name == "usb")
    {
        camera_ = std::make_unique<UsbCamera>(usb_index);
    }
    else if (camera_name == "replay")
    {
        camera_ = std::make_unique<ReplayCamera>(replay_path);
    }
    else
    {
        throw std::runtime_error("Unknown camera type:"+camera_name);
    }
}
bool Camera::open()
{
    return camera_->open();
}
cv::Mat Camera::read()
{
    cv::Mat img = camera_->read();
    if (!img.empty())
    {
       cv::flip(img, img, flip_code_); 
    }
    return img;
}
void Camera::close()
{
    camera_->close();
}
}