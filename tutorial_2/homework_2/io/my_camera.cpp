#include "my_camera.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"
#include <stdexcept>
#include "hikrobot/include/MvCameraControl.h"
#include <cstdlib>
#include <unordered_map>

namespace {

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

}  // namespace

namespace io {

    Camera::Camera(const std::string& config_path) {
        auto yaml = tools::load(config_path);
        
        auto camera_name = tools::read<std::string>(yaml, "camera_name");
        auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
        auto gain        = tools::read<double>(yaml, "gain");
        auto vid_pid     = tools::read<std::string>(yaml, "vid_pid");
        flip_code_       = tools::read<int>(yaml, "flip_code", 2);

        if (camera_name == "hikrobot") {
            camera_ = std::make_unique<HikCamera>(exposure_ms, gain, vid_pid);
        }
        else if (camera_name == "usb") {
            camera_ = std::make_unique<UsbCamera>(exposure_ms, gain, vid_pid);
        } else {
            tools::logger()->error("unknown camera_name: {}", camera_name);
            exit(1);
        }
    }

    void Camera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
        camera_->read(img, timestamp);
        if (!img.empty() && flip_code_ != 2) {
            cv::flip(img, img, flip_code_);
        }
    }

    UsbCamera::UsbCamera(double exposure_ms, double gain, const std::string& /*vid_pid*/)
    {
        cap_.open(0);
        if(!cap_.isOpened()) {
            tools::logger()->error("failed to open usb camera");
            throw std::runtime_error("usb camera open failed");
        }   
        cap_.set(cv::CAP_PROP_EXPOSURE, exposure_ms);
        cap_.set(cv::CAP_PROP_GAIN, gain);
    }

    HikCamera::HikCamera(double exposure_ms, double gain, const std::string& /*vid_pid*/)
    {
        MV_CC_DEVICE_INFO_LIST device_list;
        int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        if (ret != MV_OK || device_list.nDeviceNum == 0) {
            tools::logger()->error("no Hik camera found");
            throw std::runtime_error("no Hik camera");
        }

        ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
        if (ret != MV_OK) {
            tools::logger()->error("MV_CC_CreateHandle failed");
            throw std::runtime_error("create handle failed");
        }

        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK) {
            tools::logger()->error("MV_CC_OpenDevice failed");
            throw std::runtime_error("open device failed");
        }

        MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
        MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
        MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_ms * 1000);   // мс -> мкс
        MV_CC_SetFloatValue(handle_, "Gain", gain);
        MV_CC_SetFrameRate(handle_, 60);

        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK) {
            tools::logger()->error("MV_CC_StartGrabbing failed");
            throw std::runtime_error("start grabbing failed");
        }
    }

    HikCamera::~HikCamera() {
        if (!handle_) return;
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
    }

    void HikCamera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
        MV_FRAME_OUT raw;
        int ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
        if (ret != MV_OK) {
            img = cv::Mat();     // пустая картинка = "кадра нет"
            return;
        }
        timestamp = std::chrono::steady_clock::now();
        img = transfer(raw);
        MV_CC_FreeImageBuffer(handle_, &raw);
    }

}