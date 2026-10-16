#pragma once
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <memory>
#include <stdexcept>

namespace io {
    using TimePoint = std::chrono::steady_clock::time_point;

    //----------CameraBase-------------
    class CameraBase{
        public:
        virtual ~CameraBase() = default;
        virtual void read(cv::Mat& img, TimePoint& timestamp) =0;
        virtual bool isOpen() const =0;
    };
    //----------HikCamera--------------
    class HikCamera : public CameraBase {
        public:
        HikCamera (double exposure_ms, double gain, const std::string& vid_pid);
        ~HikCamera() override;

        HikCamera(const HikCamera&) = delete;
        HikCamera& operator=(const HikCamera&) = delete;

        void read(cv::Mat& img, TimePoint& timestamp) override;
        bool isOpen() const override;
        private:
        void close();
        void* handle_=nullptr;
        double exposure_ms_ = 0.0;
        double gain_ = 0.0;
        std::string vid_pid_;
    };
    //----------UsbCamera--------------
    class UsbCamera : public CameraBase {
        public:
        explicit UsbCamera (const std::string& vid_pid);
        ~UsbCamera () override;

        UsbCamera(const UsbCamera&) = delete;
        UsbCamera& operator=(const UsbCamera&) = delete;

        void read (cv::Mat& img, TimePoint& timestamp) override;
        bool isOpen() const override;
        private:
        cv::VideoCapture cap_;
    };
    //------------Camera---------------
    class Camera {
        public:
        explicit Camera (const std::string& config_path);
        ~Camera();

        Camera(const Camera&) = delete;                         
        Camera& operator=(const Camera&) = delete;
        void read (cv::Mat& img, TimePoint& timestamp);
        bool isOpen() const;
        private:
        std::unique_ptr<CameraBase> camera_;
        int flip_code_=-1;
    };
}