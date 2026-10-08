#include"my_camera.hpp"
#include<iostream>
#include "hikrobot/include/MvCameraControl.h"
namespace io{

HikCamera::~HikCamera() {
        close();
} 

bool  HikCamera::open(const std::string & config){
    MV_CC_DEVICE_INFO_LIST device_list ={};
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    
    std::cout << "EnumDevices ret = " << ret
          << ", nDeviceNum = " << device_list.nDeviceNum << std::endl;

    if (ret != MV_OK)  {
        std::cout<<"打开相机失败"<<std::endl;
      return false;
    }

    if (device_list.nDeviceNum == 0) {
        std::cout<<"没设备可用，打开失败"<<std::endl;
      return false;
    }
    ret = MV_CC_CreateHandle(&handle, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        std::cout<<"创建句柄失败"<<std::endl;
      return false;
    }
  
    ret = MV_CC_OpenDevice(handle);
    if (ret != MV_OK) { 
        std::cout<<"建立通信连接失败,打开相机失败"<<std::endl;
      return false;
    }
  
    MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle, "ExposureTime", 10000); 
    MV_CC_SetFloatValue(handle, "Gain", 20);
    MV_CC_SetFrameRate(handle, 60);  
    std::cout<<"建立通信连接,打开相机"<<std::endl;
    ret = MV_CC_StartGrabbing(handle);  // 开始抓取图像
    return true;
}

cv::Mat HikCamera::transfer(MV_FRAME_OUT& raw) // 将海康原始图像数据转换为 OpenCV Mat 格式
{
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;    //海康威视 MVS SDK 中用于图像像素格式转换的结构体
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight; 

    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
      {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
      {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
      {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
      {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(pixel_type));
    
    return img;
}
 
bool HikCamera::read(cv::Mat& frame,std::chrono::steady_clock::time_point & timestamp){

    if (ret != MV_OK) {
      return false;
    }
  
    MV_FRAME_OUT raw;
    unsigned int nMsec = 100; 

    ret = MV_CC_GetImageBuffer(handle, &raw, nMsec);
    if (ret != MV_OK) {
      return false;
    }
   
    frame = transfer(raw);

    ret = MV_CC_FreeImageBuffer(handle, &raw); 
    if (ret != MV_OK) {
      return false;
    }
    return true;
}

bool HikCamera::close(){
    int ret;
    ret = MV_CC_StopGrabbing(handle);
    if (ret != MV_OK) {
        return false;
    }

    ret = MV_CC_CloseDevice(handle);
    if (ret != MV_OK) {
        return false;
    }

    ret = MV_CC_DestroyHandle(handle);
    if (ret != MV_OK) {
        return false;
    }
    return true;
}


bool UsbCamera::open(const std::string & config){
  this->cap.open(0, cv::CAP_V4L2);
    if (!this->cap.isOpened()) {
        std::cerr << "UsbCamera isn't open" << std::endl;
        return false;
    }
    std::cout << "UsbCamera is open" << std::endl;
    return true;
}

bool UsbCamera::read(cv::Mat& frame,std::chrono::steady_clock::time_point & timestamp){
  timestamp = std::chrono::steady_clock::now();
  cap.read(frame);
  if (frame.empty()){std::cout<<"1"<<std::endl;return false;}
  return true;
}

bool UsbCamera::close(){
    if(cap.isOpened()){
        cap.release();
        std::cout<<"UsbCamera is closed"<<std::endl;
        return true;
    }
    return false;
}

bool ReplayCamera::open(const std::string & config){
  //cap_ = std::make_unique<cv::VideoCapture>("/home/rsve/test/test1.mp4");
  cap_ = std::make_unique<cv::VideoCapture>("/home/rsve/test/SPR_Vision_Tutorial_27/tutorial_2/Study/demo.mp4");
  //cv::VideoCapture cap_("/home/rsve/test/test.mp4");
  return true;
}
bool ReplayCamera::read(cv::Mat& frame,std::chrono::steady_clock::time_point & timestamp){
  timestamp = std::chrono::steady_clock::now();
  cap_ -> read(frame);
  if (frame.empty()){std::cout<<"2"<<std::endl;return false;}
  return true;
}

bool ReplayCamera::close(){
    if (cap_) {
        cap_->release();
        cap_.reset();
    }
    return true; 
}

//Camera
Camera::Camera(const std::string & config) {
     if (config == "hik") {
    impl = std::make_unique<HikCamera>();
  }
     if (config == "usb") {
    impl = std::make_unique<UsbCamera>();
}
if (config == "replay") {
    impl = std::make_unique<ReplayCamera>();
}
     impl->open(config);
}
}