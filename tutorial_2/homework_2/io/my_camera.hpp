#pragma once
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <memory>

namespace io {
    class CameraBase {
        public: 
            virtual ~CameraBase() = default;
            virtual void read(cv::Mat& img,
                                std::chrono::steady_clock::time_point& timestamp) = 0;
    };

    class HikCamera : public CameraBase {
        public:
            HikCamera(double exposure_ms, double gain, const std::string& vid_pid);
            ~HikCamera() override;
            void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;
        private:
            void* handle_ = nullptr;
    };


    class UsbCamera : public CameraBase {
        public:
            UsbCamera(double exposure_ms, double gain, const std::string& vid_pid);
            // ~UsbCamera() override;
            void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;
        private:
            cv::VideoCapture cap_;
    };

    class Camera {
        public: 
            explicit Camera(const std::string& config_path);
            void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp);
        private: 
            std::unique_ptr<CameraBase> camera_;
            int flip_code_;
    };
}