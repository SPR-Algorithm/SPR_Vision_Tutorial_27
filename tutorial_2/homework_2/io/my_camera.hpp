#ifndef IO_MY_CAMERA_HPP
#define IO_MY_CAMERA_HPP
#include <memory>
#include <string>
#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

namespace io {
    class CameraBase
    {
        public:
            virtual ~CameraBase() = default;
            virtual bool open() = 0;
            virtual bool read(cv::Mat &img) = 0;
            virtual void close() = 0;
    };
    class HikCamera : public CameraBase
    {
        public:
            explicit HikCamera(const YAML::Node &config);
            ~HikCamera() override;

            bool open() override;
            bool read(cv::Mat &img) override;
            void close() override;
        private:
            YAML::Node config_;
            void *handle_=nullptr;
            bool opened_=false;
    };
    class UsbCamera : public CameraBase
    {
        public:
            explicit UsbCamera(const YAML::Node &config);
            ~UsbCamera() override;

            bool open() override;
            bool read(cv::Mat &img) override;
            void close() override;
        private:
            YAML::Node config_;
            cv::VideoCapture capture_;
    };
    class Camera
    {
        public:
            explicit Camera(const std::string &config_path);
            ~Camera();

            bool read(cv::Mat &img);
            void close();
        private:
            std::unique_ptr<CameraBase> camera_;
    };
}//namespace io
    
#endif // IO_MY_CAMERA_HPP