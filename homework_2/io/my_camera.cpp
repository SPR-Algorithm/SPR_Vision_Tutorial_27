#include "my_camera.hpp"
#include "MvCameraControl.h"

#undef major
#undef minor
#undef atomic
#undef bytes

#include "../tools/logger.hpp"
#include <spdlog/spdlog.h>

struct HikCamera::Impl {
    void* handle = nullptr;
};

HikCamera::HikCamera(int exposure_ms, double gain, int flip_code)
   : impl_(std::make_unique<Impl>()),
     exposure_ms_(exposure_ms),
     gain_(gain),
     flip_code_(flip_code) {}


HikCamera::~HikCamera() {
    close();
}

bool HikCamera::open() {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0,
sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet =
MV_CC_EnumDevices(MV_GIGE_DEVICE |
MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet ||
stDeviceList.nDeviceNum == 0) {
        SPDLOG_ERROR("找不到海康相机设备！");
        return false;
    }
    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[0];
    nRet = MV_CC_CreateHandle(&impl_->handle, pDeviceInfo);
    if (MV_OK != nRet) return false;
    nRet = MV_CC_OpenDevice(impl_->handle);
    if (MV_OK != nRet) return false;

    MV_CC_SetEnumValue(impl_->handle,
"ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetFloatValue(impl_->handle,
"ExposureTime", exposure_ms_* 1000.0f);
    MV_CC_SetFloatValue(impl_->handle,
"Gain", gain_);

    nRet = MV_CC_StartGrabbing(impl_->handle);
    if (MV_OK != nRet) return false;
    SPDLOG_INFO("海康相机 {} 打开成功！", name());
    return true;
}

bool HikCamera::getFrame(cv::Mat& frame) {
    if (!impl_->handle) return false;

    MV_FRAME_OUT_INFO_EX stImageInfo;
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char* pData = new unsigned char[1920 * 1080 * 3];
    int nRet = MV_CC_GetOneFrameTimeout(impl_->handle, pData, 1920 * 1080 * 3, &stImageInfo, 1000);
    if (MV_OK != nRet) {
        delete[] pData;
        return false;
    }
    if (stImageInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
        cv::Mat raw_img(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
        raw_img.copyTo(frame);
    } else {
        delete[] pData;
        return false;
    }
    delete[] pData;

    if (flip_code_ == 1) cv::flip(frame, frame, 1);
    else if (flip_code_ == 0) cv::flip(frame, frame, 0);
    else if (flip_code_ == -1) cv::flip(frame, frame, -1);

    return true;
}

void HikCamera::close() {
    if (!impl_->handle) return;
    MV_CC_StopGrabbing(impl_->handle);
    MV_CC_CloseDevice(impl_->handle);
    MV_CC_DestroyHandle(impl_->handle);
    impl_->handle = nullptr;

    spdlog::info("海康相机已关闭");
}

UsbCamera::UsbCamera(int index, int exposure_ms, double gain, int flip_code)
    : index_(index), exposure_ms_(exposure_ms), gain_(gain), flip_code_(flip_code) {}
UsbCamera::~UsbCamera() {
    close();
}

bool UsbCamera::open() {
    cap_.open(index_);
    if (!cap_.isOpened()) {
        SPDLOG_ERROR("USB相机打开失败");
        return false;
    }
    SPDLOG_INFO("USB相机 {} 打开成功！", name());
    return true;
}

bool UsbCamera::getFrame(cv::Mat& frame) {
    if (!cap_.isOpened()) return false;
    cap_ >> frame;
    if (frame.empty()) return false;

    if (flip_code_ == 1) cv::flip(frame, frame, 1);
    else if (flip_code_ == 0) cv::flip(frame, frame, 0);
    else if (flip_code_ == -1) cv::flip(frame, frame, -1);

    return true;
}

void UsbCamera::close() {
    if (cap_.isOpened()) cap_.release();
    SPDLOG_INFO("USB相机已关闭");
}

