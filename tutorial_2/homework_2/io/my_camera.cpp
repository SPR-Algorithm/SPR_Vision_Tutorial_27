#include "my_camera.hpp"

#include <cstdio>
#include <unordered_map>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

// 海康 SDK：只在这个 .cpp 里出现（PIMPL）。
#include "hikrobot/include/MvCameraControl.h"

namespace io {

// ============================================================
// HikCamera
// ============================================================
struct HikCamera::Impl {
    void* handle = nullptr;
    bool  opened = false;
};

HikCamera::HikCamera() : impl_(std::make_unique<Impl>()) {}
HikCamera::~HikCamera() { close(); }

/// 照抄 example.cpp 里的 transfer()：把海康的 Bayer 裸数据转成 RGB。
/// 注意：返回的 Mat 是对 raw 缓冲的包装，调用方在 FreeImageBuffer 之前必须用完。
static cv::Mat hik_transfer(MV_FRAME_OUT& raw) {
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight),
                CV_8UC1, raw.pBufAddr);

    static const std::unordered_map<int, int> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB},
    };
    auto it = type_map.find(raw.stFrameInfo.enPixelType);
    if (it != type_map.end()) {
        cv::cvtColor(img, img, it->second);
    }
    return img;
}

bool HikCamera::open(const YAML::Node& cfg) {
    std::string vid_pid_str = tools::read<std::string>(cfg, "vid_pid", std::string(""));
    double exposure_ms = tools::read<double>(cfg, "exposure_ms", 3.0);
    double gain        = tools::read<double>(cfg, "gain", 0.0);

    // 1) 枚举 USB 设备
    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof(device_list));
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        tools::logger()->error("hik: no USB device, ret={}", ret);
        return false;
    }

    // 2) 按 vid_pid（格式 "2bdf:0001"）匹配设备；没配就拿第一台
    unsigned int want_vid = 0, want_pid = 0;
    if (sscanf(vid_pid_str.c_str(), "%x:%x", &want_vid, &want_pid) != 2) {
        want_vid = want_pid = 0;
    }

    MV_CC_DEVICE_INFO* picked = nullptr;
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
        auto* info = device_list.pDeviceInfo[i];
        if (want_vid == 0 && want_pid == 0) { picked = info; break; }
        if (info->nTLayerType == MV_USB_DEVICE) {
            auto& usb = info->SpecialInfo.stUsb3VInfo;
            if (usb.idVendor == want_vid && usb.idProduct == want_pid) {
                picked = info;
                break;
            }
        }
    }
    if (!picked) {
        tools::logger()->error("hik: no device matching vid_pid={}", vid_pid_str);
        return false;
    }

    // 3) 创建句柄 + 打开
    ret = MV_CC_CreateHandle(&impl_->handle, picked);
    if (ret != MV_OK) { tools::logger()->error("hik: CreateHandle ret={}", ret); return false; }

    ret = MV_CC_OpenDevice(impl_->handle);
    if (ret != MV_OK) { tools::logger()->error("hik: OpenDevice ret={}", ret); return false; }

    // 4) 关掉自动曝光 / 自动增益，白平衡保持连续
    MV_CC_SetEnumValue(impl_->handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(impl_->handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(impl_->handle, "GainAuto", MV_GAIN_MODE_OFF);

    // 5) 曝光（海康单位 us）和增益
    MV_CC_SetFloatValue(impl_->handle, "ExposureTime", static_cast<float>(exposure_ms * 1000.0));
    MV_CC_SetFloatValue(impl_->handle, "Gain", static_cast<float>(gain));
    MV_CC_SetFrameRate(impl_->handle, 60);

    // 6) 开始取流
    ret = MV_CC_StartGrabbing(impl_->handle);
    if (ret != MV_OK) { tools::logger()->error("hik: StartGrabbing ret={}", ret); return false; }

    impl_->opened = true;
    tools::logger()->info("hik camera opened, exposure={}ms gain={}", exposure_ms, gain);
    return true;
}

void HikCamera::close() {
    if (!impl_->handle) return;
    if (impl_->opened) {
        MV_CC_StopGrabbing(impl_->handle);
        MV_CC_CloseDevice(impl_->handle);
    }
    MV_CC_DestroyHandle(impl_->handle);
    impl_->handle = nullptr;
    impl_->opened = false;
}

bool HikCamera::read(cv::Mat& img) {
    if (!impl_->opened) return false;

    MV_FRAME_OUT raw;
    memset(&raw, 0, sizeof(raw));
    unsigned int nMsec = 100;
    int ret = MV_CC_GetImageBuffer(impl_->handle, &raw, nMsec);
    if (ret != MV_OK) {
        tools::logger()->warn("hik: GetImageBuffer ret={}", ret);
        return false;
    }

    // 深拷贝一份，FreeImageBuffer 之后 img 仍然有效
    img = hik_transfer(raw).clone();

    MV_CC_FreeImageBuffer(impl_->handle, &raw);
    return !img.empty();
}

bool HikCamera::isOpened() const { return impl_->opened; }

// ============================================================
// UsbCamera
// ============================================================
UsbCamera::UsbCamera() = default;
UsbCamera::~UsbCamera() { close(); }

bool UsbCamera::open(const YAML::Node& cfg) {
    // USB 模式下 vid_pid 当设备号用：0 = /dev/video0
    int index = tools::read<int>(cfg, "vid_pid", 0);
    cap_.open(index, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        tools::logger()->error("usb: cannot open /dev/video{}", index);
        return false;
    }
    tools::logger()->info("usb camera {} opened", index);
    return true;
}

void UsbCamera::close() { if (cap_.isOpened()) cap_.release(); }
bool UsbCamera::read(cv::Mat& img) { return cap_.read(img) && !img.empty(); }
bool UsbCamera::isOpened() const { return cap_.isOpened(); }

// ============================================================
// ReplayCamera（加分项）
// ============================================================
ReplayCamera::ReplayCamera() = default;
ReplayCamera::~ReplayCamera() { close(); }

bool ReplayCamera::open(const YAML::Node& cfg) {
    std::string path = tools::read<std::string>(cfg, "replay_path", std::string("test.mp4"));
    cap_.open(path);
    if (!cap_.isOpened()) {
        tools::logger()->error("replay: cannot open '{}'", path);
        return false;
    }
    tools::logger()->info("replay '{}' opened", path);
    return true;
}

void ReplayCamera::close() { if (cap_.isOpened()) cap_.release(); }
bool ReplayCamera::read(cv::Mat& img) { return cap_.read(img) && !img.empty(); }
bool ReplayCamera::isOpened() const { return cap_.isOpened(); }

// ============================================================
// 外观类 Camera
// ============================================================
Camera::Camera(const YAML::Node& cfg) {
    std::string name = tools::read<std::string>(cfg, "camera_name", std::string("hikrobot"));

    if (name == "hikrobot" || name == "hik") {
        impl_ = std::make_unique<HikCamera>();
    } else if (name == "usb") {
        impl_ = std::make_unique<UsbCamera>();
    } else if (name == "replay") {
        impl_ = std::make_unique<ReplayCamera>();
    } else {
        tools::logger()->error("camera_name '{}' unknown, fallback to hikrobot", name);
        impl_ = std::make_unique<HikCamera>();
    }

    if (!impl_->open(cfg)) {
        tools::logger()->error("camera '{}' failed to open", name);
    }

    flip_code_ = tools::read<int>(cfg, "flip_code", 1);
}

Camera::~Camera() { close(); }

bool Camera::read(cv::Mat& img) {
    if (!impl_ || !impl_->read(img)) return false;
    if (flip_code_ != 0) {
        // cv::flip: >0 左右镜像, 0 上下, -1 双。yaml 里 flip_code=2 等价于左右镜像。
        cv::flip(img, img, flip_code_ > 0 ? 1 : flip_code_);
    }
    return true;
}

bool Camera::isOpened() const { return impl_ && impl_->isOpened(); }
void Camera::close() { if (impl_) impl_->close(); }

}  // namespace io