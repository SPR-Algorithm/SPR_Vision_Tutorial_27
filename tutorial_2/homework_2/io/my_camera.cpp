#include "io/my_camera.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"
#include "hikrobot/include/MvCameraControl.h"
#include <iostream>
#include <unordered_map>

namespace io {

    // 1. 把 example.cpp 最上面的 transfer 函数原封不动搬到这里
    cv::Mat transfer(MV_FRAME_OUT& raw)
    {
        MV_CC_PIXEL_CONVERT_PARAM cvt_param;
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

    // ---------------- HikCamera 实现 ----------------
    HikCamera::~HikCamera() {
        if (handle_) {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
        }
    }

    bool HikCamera::init(const std::string& config_path) {
        auto config = tools::load(config_path);
        double exposure = tools::read<double>(config, "exposure_ms");
        double gain = tools::read<double>(config, "gain");

        // 2. 把 example.cpp 里创建句柄、打开设备、设置参数的代码搬到这里
        int ret;
        MV_CC_DEVICE_INFO_LIST device_list;
        ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        if (ret != MV_OK || device_list.nDeviceNum == 0) {
            tools::logger()->error("找不到海康相机设备！");
            return false;
        }

        ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
        if (ret != MV_OK) {
            tools::logger()->error("创建句柄失败！");
            return false;
        }

        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK) {
            tools::logger()->error("打开相机失败！");
            return false;
        }

        // 用 yaml 读出的曝光和增益替换掉 example 里写死的 10000 和 20
        MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
        MV_CC_SetFloatValue(handle_, "ExposureTime", exposure);
        MV_CC_SetFloatValue(handle_, "Gain", gain);
        MV_CC_SetFrameRate(handle_, 60);

        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) {
            tools::logger()->error("开始取流失败！");
            return false;
        }
        
        tools::logger()->info("海康相机初始化成功");
        return true; 
    }

    bool HikCamera::read(cv::Mat& img, double& timestamp) {
        // 3. 把 example.cpp 里获取缓冲区、转换图像的代码搬到这里
        MV_FRAME_OUT raw;
        unsigned int nMsec = 100;

        int ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
        if (ret != MV_OK) {
            tools::logger()->error("获取图像失败！");
            return false;
        }

        img = transfer(raw); // 调用刚才搬过来的转换函数

        ret = MV_CC_FreeImageBuffer(handle_, &raw); // 释放缓冲区
        if (ret != MV_OK) {
            tools::logger()->error("释放缓冲区失败！");
            return false;
        }

        timestamp = cv::getTickCount() / cv::getTickFrequency();
        return true;
    }

    // ---------------- UsbCamera 实现 ----------------
    UsbCamera::~UsbCamera() {
        if (cap_.isOpened()) cap_.release();
    }

    bool UsbCamera::init(const std::string& config_path) {
        cap_.open(0); 
        if (!cap_.isOpened()) {
            tools::logger()->error("视频文件打开失败！请确认 assets/demo.mp4 存在！");
            return false;
        }
        tools::logger()->info("USB相机初始化成功（读取视频文件）");
        return true;
    }

    bool UsbCamera::read(cv::Mat& img, double& timestamp) {
        if (!cap_.isOpened()) return false;
        cap_ >> img;
        timestamp = cv::getTickCount() / cv::getTickFrequency();
        return !img.empty();
    }

    // ---------------- Camera 外观类实现 ----------------
    Camera::Camera() {}
    Camera::~Camera() { delete camera_impl_; }

    bool Camera::init(const std::string& config_path) {
        auto config = tools::load(config_path);
        std::string name = tools::read<std::string>(config, "camera_name");
        
        if (name == "hikrobot") {
            camera_impl_ = new HikCamera();
        } else {
            camera_impl_ = new UsbCamera();
        }
        return camera_impl_->init(config_path);
    }

    bool Camera::read(cv::Mat& img, double& timestamp) {
    if (camera_impl_) return camera_impl_->read(img, timestamp);
    return false;
}}