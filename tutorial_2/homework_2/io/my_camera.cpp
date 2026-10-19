#include "my_camera.hpp"
#include "hikrobot/include/MvCameraControl.h"
#include "../tools/yaml.hpp"
#include <iostream>
#include <unordered_map>
#include <vector>

namespace io
{
    UsbCamera::UsbCamera(const YAML::Node &config) : config_(config) {}
    bool UsbCamera::open()
    {
        if (!capture_.open(0))
        {
            std::cerr << "[UsbCamera] Failed to open camera 0\n";
            return false;
        }

        std::cout << "[UsbCamera] Camera opened\n";
        return true;
    }
    bool UsbCamera::read(cv::Mat &img)
    {
        if(!capture_.isOpened()){
            return false;
        }
        return capture_.read(img);
    }
    void UsbCamera::close()
    {
        if(capture_.isOpened()){
            capture_.release();
        }
    }
    HikCamera::HikCamera(const YAML::Node& config): config_(config){}

    HikCamera::~HikCamera()
    {
        close();
    }

    bool HikCamera::open()
    {
        MV_CC_DEVICE_INFO_LIST device_list;
        int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);

        if (ret != MV_OK || device_list.nDeviceNum == 0)
        {
            return false;
        }

        ret = MV_CC_CreateHandle(
            &handle_, device_list.pDeviceInfo[0]);

        if (ret != MV_OK)
        {
            handle_ = nullptr;
            return false;
        }

        ret = MV_CC_OpenDevice(handle_);
        if (ret != MV_OK)
        {
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return false;
        }

        MV_CC_SetEnumValue(
            handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        MV_CC_SetEnumValue(
            handle_, "GainAuto", MV_GAIN_MODE_OFF);

        float exposure_ms = config_["exposure_ms"].as<float>();
        float gain = config_["gain"].as<float>();

        MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_ms * 1000.0f);
        MV_CC_SetFloatValue(handle_, "Gain", gain);

        ret = MV_CC_StartGrabbing(handle_);
        if (ret != MV_OK)
        {
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            handle_ = nullptr;
            return false;
        }

        opened_ = true;
        return true;
    }
    bool HikCamera::read(cv::Mat& img)
    {
        if (!opened_ || handle_ == nullptr)
        {
            return false;
        }

        MV_FRAME_OUT raw{};

        int ret = MV_CC_GetImageBuffer(handle_, &raw, 1000);
        if (ret != MV_OK)
        {
            return false;
        }

        const int width = raw.stFrameInfo.nWidth;
        const int height = raw.stFrameInfo.nHeight;

        std::vector<unsigned char> buffer(
            static_cast<size_t>(width) * height * 3);

        MV_CC_PIXEL_CONVERT_PARAM param{};
        param.nWidth = width;
        param.nHeight = height;
        param.pSrcData = raw.pBufAddr;
        param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
        param.enSrcPixelType = raw.stFrameInfo.enPixelType;
        param.pDstBuffer = buffer.data();
        param.nDstBufferSize = static_cast<unsigned int>(buffer.size());
        param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

        ret = MV_CC_ConvertPixelType(handle_, &param);

        int free_ret = MV_CC_FreeImageBuffer(handle_, &raw);

        if (ret != MV_OK || free_ret != MV_OK)
        {
            return false;
        }

        img = cv::Mat(height, width, CV_8UC3, buffer.data()).clone();

        return true;
    }

    void HikCamera::close()
    {
        if (handle_ == nullptr)
        {
            opened_ = false;
            return;
        }

        if (opened_)
        {
            MV_CC_StopGrabbing(handle_);
            opened_ = false;
        }

        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
    Camera::~Camera() { close(); }
    UsbCamera::~UsbCamera() { close(); }

    Camera::Camera(const std::string &config_path)
    {
        YAML::Node config = tools::load(config_path);
        std::string camera_name =
        tools::read<std::string>(config, "camera_name");

        if (camera_name == "usb") {
            camera_ = std::make_unique<UsbCamera>(config);
        } else if (camera_name == "hikrobot") {
            camera_ = std::make_unique<HikCamera>(config);
        } else {
            std::cerr << "[Camera] Unknown camera name: "
                  << camera_name << '\n';
        return;
        }

        if (!camera_->open()) {
            std::cerr << "[Camera] Failed to open camera\n";
            camera_.reset();
        }
    }

    bool Camera::read(cv::Mat &img)
    {
        if (!camera_) {
         return false;
        }

        return camera_->read(img);
    }

    void Camera::close()
    {
        if (camera_) {
        camera_->close();
        }
    }
}//namespace io

