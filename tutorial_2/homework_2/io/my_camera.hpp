#pragma once
#include <opencv2/opencv.hpp>
#include<memory>
#include <string>

namespace io
{
    class CameraBase
    {
        public:
            virtual bool open()=0;
            virtual cv::Mat read()=0;
            virtual void close()=0;
            virtual ~CameraBase()=default;
    };
class HikCamera : public CameraBase
{
    public:
        HikCamera(double exposure_ms,double gain,const std::string& vid_pid);
        ~HikCamera() override { close(); }
        bool open() override;
        cv::Mat read()override;
        void close()override;
    private:
        void* handle_{nullptr};
        double exposure_ms_;
        double gain_;
        std::string vid_pid_;
};
class UsbCamera: public CameraBase
{
    public:
        explicit UsbCamera(int index);
        bool open() override;
        cv::Mat read() override;
        void close() override;
    private:
        int index_;
        cv::VideoCapture cap_;
};
class ReplayCamera : public CameraBase
{
    public:
        explicit ReplayCamera(const std::string& path);
        bool open() override;
        cv::Mat read() override;
        void close() override;
    private:
        std::string path_;
        cv::VideoCapture cap_;
};
class Camera
{
    public:
        Camera(const std::string& config_path);
        bool open();
        cv::Mat read();
        void close();
    private:
        std::unique_ptr<CameraBase> camera_;
        int flip_code_{1};
};
}