#include "io/my_camera.hpp"
#include "hikrobot/include/MvCameraControl.h"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

namespace io
{
    namespace
    {
        std::pair<int, int> parse_vid_pid(const std::string &vid_pid)
        {
            if (vid_pid.empty())
            {
                return {-1, -1};
            }
            const auto pos = vid_pid.find(':');
            if (pos == std::string::npos)
            {
                return {-1, -1};
            }
            try
            {
                const int vid = std::stoi(vid_pid.substr(0, pos), nullptr, 16);
                const int pid = std::stoi(vid_pid.substr(pos + 1), nullptr, 16);
                return {vid, pid};
            }
            catch (const std::exception &)
            {
                return {-1, -1};
            }
        }
        int find_usb_index(const std::string &vid_pid)
        {
            const auto [target_vid, target_pid] = parse_vid_pid(vid_pid);
            if (target_vid < 0)
                return 0;

            std::error_code ec;
            const std::filesystem::path root("/sys/class/video4linux");
            if (!std::filesystem::exists(root, ec))
                return 0;

            for (const auto &entry : std::filesystem::directory_iterator(root, ec))
            {
                const std::string name = entry.path().filename().string();
                if (name.rfind("video", 0) != 0)
                    continue;

                const auto dev = entry.path() / "device";

                auto read_hex = [&dev](const char *file) -> int
                {
                    std::ifstream in((dev / file).string());
                    std::string value;
                    if (!(in >> value))
                        return -1;
                    try
                    {
                        return std::stoi(value, nullptr, 16);
                    }
                    catch (const std::exception &)
                    {
                        return -1;
                    }
                };

                if (read_hex("idVendor") == target_vid &&
                    read_hex("idProduct") == target_pid)
                {
                    return std::stoi(name.substr(5));
                }
            }
            return 0;
        }

        cv::Mat frame_to_bgr(void *handle, MV_FRAME_OUT &raw)
        {
            (void)handle;
            cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight),
                        CV_8U, raw.pBufAddr);

            auto pixel_type = raw.stFrameInfo.enPixelType;
            const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
                {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
                {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
                {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
                {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR}};

            auto it = type_map.find(pixel_type);
            if (it == type_map.end())
            {
                return cv::Mat{};
            }

            cv::Mat bgr;
            cv::cvtColor(img, bgr, it->second);
            return bgr.clone();
        }

    } // namespace

    //=======================HikCamera=====================

    HikCamera::HikCamera(double exposure_ms, double gain, const std::string &vid_pid) : exposure_ms_(exposure_ms), gain_(gain), vid_pid_(vid_pid)
    {
        MV_CC_DEVICE_INFO_LIST device_list = {};
        int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
        if (ret != MV_OK)
            throw std::runtime_error("[HikCamera] EnumDevices failed");
        if (device_list.nDeviceNum == 0)
            throw std::runtime_error("[HikCamera] No camera");

        int index = 0;
        const auto [vid, pid] = parse_vid_pid(vid_pid);
        if (vid >= 0 && pid >= 0)
        {
            for (unsigned int i = 0; i < device_list.nDeviceNum; ++i)
            {
                const auto *info = device_list.pDeviceInfo[i];
                if (info->nTLayerType != MV_USB_DEVICE)
                    continue;
                const auto &usb = info->SpecialInfo.stUsb3VInfo;
                if (usb.idVendor == vid && usb.idProduct == pid)
                {
                    index = i;
                    break;
                }
            }
        }

        ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[index]);
        if (ret != MV_OK)
        {
            handle_ = nullptr;
            throw std::runtime_error("[HikCamera] CreateHandle failed");
        }

        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK)
        {
            close();
            throw std::runtime_error("[HikCamera] OpenDevice failed");
        }

        MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
        MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
        MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_ms * 1000.0f);
        MV_CC_SetFloatValue(handle_, "Gain", gain);
        MV_CC_SetFrameRate(handle_, 60);

        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK)
        {
            close();
            throw std::runtime_error("[HikCamera] StartGrabbing failed");
        }
        tools::logger()->info("[HikCamera] Opened: vid_pid={}", vid_pid_);
    }
    HikCamera::~HikCamera()
    {
        close();
    }
    void HikCamera::close()
    {
        if (handle_ == nullptr)
            return;
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
    void HikCamera::read(cv::Mat &img, TimePoint &timestamp)
    {
        img = cv::Mat{};
        if (handle_ == nullptr)
            return;
        MV_FRAME_OUT raw = {};
        if (MV_CC_GetImageBuffer(handle_, &raw, 100) != MV_OK)
            return;

        img = frame_to_bgr(handle_, raw);
        MV_CC_FreeImageBuffer(handle_, &raw);

        if (!img.empty())
        {
            timestamp = std::chrono::steady_clock::now();
        }
    }
    bool HikCamera::isOpen() const
    {
        return handle_ != nullptr;
    }

    //=======================UsbCamera======================

    UsbCamera::UsbCamera(const std::string &vid_pid)
    {
        const int resolved = find_usb_index(vid_pid);
        std::vector<int> candidates{resolved};
        for (int i = 0; i <= 5; ++i)
        {
            if (i != resolved)
                candidates.push_back(i);
        }

        for (const int index : candidates)
        {
            if (cap_.open(index))
            {
                tools::logger()->info("[UsbCamera] Opened: /dev/video{}", index);
                return;
            }
        }
        throw std::runtime_error("[UsbCamera] Cannot open camera");
    }
    UsbCamera::~UsbCamera()
    {
        if (cap_.isOpened())
        {
            cap_.release();
        }
    }
    void UsbCamera::read(cv::Mat &img, TimePoint &timestamp)
    {
        img = cv::Mat{};
        if (!cap_.isOpened())
            return;
        if (!cap_.read(img) || img.empty())
        {
            img = cv::Mat{};
            return;
        }
        timestamp = std::chrono::steady_clock::now();
    }
    bool UsbCamera::isOpen() const
    {
        return cap_.isOpened();
    }
    //===========================Camera==========================
    Camera::Camera(const std::string &config_path)
    {
        auto yaml = tools::load(config_path);
        auto camera_name = tools::read<std::string>(yaml, "camera_name");
        auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
        auto gain = tools::read<double>(yaml, "gain");
        auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
        flip_code_ = tools::read<int>(yaml, "flip_code", -1);

        if (camera_name == "hikrobot")
        {
            camera_ = std::make_unique<HikCamera>(exposure_ms, gain, vid_pid);
        }
        else if (camera_name == "usb")
        {
            camera_ = std::make_unique<UsbCamera>(vid_pid);
        }
        else
        {
            throw std::runtime_error("[Camera] Unknown camera: " + camera_name);
        }
        tools::logger()->info("Camera: '{}' created, flip_code = {}", camera_name, flip_code_);
    }
    Camera::~Camera() = default;

    void Camera::read(cv::Mat &img, TimePoint &timestamp)
    {
        camera_->read(img, timestamp);
        if (img.empty())
            return;
        if (flip_code_ == 2)
        {
            cv::rotate(img, img, cv::ROTATE_180);
        }
        else if (flip_code_ >= 0 && flip_code_ <= 1)
        {
            cv::flip(img, img, flip_code_);
        }
    }

    bool Camera::isOpen() const
    {
        return camera_ && camera_->isOpen();
    }

}