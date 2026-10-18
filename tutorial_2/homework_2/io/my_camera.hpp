#pragma once
#include <string>
#include <opencv2/opencv.hpp>

namespace io {
    class CameraBase {
    public:
        virtual ~CameraBase() = default;
        virtual bool init(const std::string& config_path) = 0; 
        virtual bool read(cv::Mat& img, double& timestamp) = 0; // 纯虚函数，规定必须能读图
    };
    class HikCamera : public CameraBase {
    public:
        HikCamera() = default;
        ~HikCamera() override;
        bool init(const std::string& config_path);
        bool read(cv::Mat& img, double& timestamp) override;
    private:
        void* handle_ = nullptr; 
    };
    class UsbCamera : public CameraBase {
    public:
        UsbCamera() = default;
        ~UsbCamera() override;
        bool init(const std::string& config_path);
        bool read(cv::Mat& img, double& timestamp) override;
    private:
        cv::VideoCapture cap_;
    };

    class Camera {
    public:
        Camera();
        ~Camera();
        bool init(const std::string& config_path);
        bool read(cv::Mat& img, double& timestamp);
    private:
        CameraBase* camera_impl_ = nullptr; 
    };
}