#include "my_camera.hpp"

#include "hikrobot/include/MvCameraControl.h"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

#include <stdexcept>
#include <unordered_map>

namespace io
{

// ---------- UsbCamera ----------

UsbCamera::UsbCamera(int index)
{
    cap_.open(index);
}

bool UsbCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
    cap_.read(img);
    timestamp = std::chrono::steady_clock::now();
    return !img.empty();
}

// ---------- HikCamera ----------

// 把 SDK 返回的原始帧转成彩色 cv::Mat（Bayer 图 -> RGB）
static cv::Mat transfer(MV_FRAME_OUT & raw)
{
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
    cv::cvtColor(img, img, type_map.at(pixel_type));

    return img;
}

HikCamera::HikCamera(double exposure_ms, double gain, const std::string & vid_pid)
{
    vid_pid_ = vid_pid;

    MV_CC_DEVICE_INFO_LIST device_list;
    if (MV_CC_EnumDevices(MV_USB_DEVICE, &device_list) != MV_OK) return;
    if (device_list.nDeviceNum == 0) return;

    if (MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]) != MV_OK) return;
    if (MV_CC_OpenDevice(handle_) != MV_OK) return;

    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_ms * 1000);  // SDK 单位是微秒，yaml 里是毫秒
    MV_CC_SetFloatValue(handle_, "Gain", gain);
    MV_CC_SetFrameRate(handle_, 60);

    MV_CC_StartGrabbing(handle_);

    tools::logger()->info("HikCamera opened, vid_pid: {}", vid_pid_);
}

bool HikCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
    MV_FRAME_OUT raw;
    if (MV_CC_GetImageBuffer(handle_, &raw, 100) != MV_OK) return false;

    img = transfer(raw);
    timestamp = std::chrono::steady_clock::now();

    MV_CC_FreeImageBuffer(handle_, &raw);
    return !img.empty();
}

HikCamera::~HikCamera()
{
    if (handle_ != nullptr) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
    }
}

// ---------- Camera 外观类 ----------

Camera::Camera(const std::string & config_path)
{
    auto yaml = tools::load(config_path);
    std::string name = tools::read<std::string>(yaml, "camera_name");
    flip_code_ = tools::read<int>(yaml, "flip_code");

    if (name == "usb") {
        camera_ = std::make_unique<UsbCamera>(0);   // 0 = 默认摄像头
    } else if (name == "hikrobot") {
        double exposure_ms = tools::read<double>(yaml, "exposure_ms");
        double gain = tools::read<double>(yaml, "gain");
        std::string vid_pid = tools::read<std::string>(yaml, "vid_pid");
        camera_ = std::make_unique<HikCamera>(exposure_ms, gain, vid_pid);
    } else {
        throw std::runtime_error("未知的 camera_name: " + name);
    }
}

bool Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
    bool ok = camera_->read(img, timestamp);
    if (ok) {
        cv::flip(img, img, flip_code_);
    }
    return ok;
}

}  // namespace io
