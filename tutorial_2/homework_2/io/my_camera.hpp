#pragma once

#include <opencv2/opencv.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace io
{

// 抽象基类：规定所有相机都要能取图
class CameraBase
{
public:
    virtual ~CameraBase() = default;

    virtual bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

// 普通 USB 摄像头（用 OpenCV 的 VideoCapture）
class UsbCamera : public CameraBase
{
public:
    explicit UsbCamera(int index);
    bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
    cv::VideoCapture cap_;
};

// 海康机器人相机（封装 SDK）
class HikCamera : public CameraBase
{
public:
    HikCamera(double exposure_ms, double gain, const std::string & vid_pid);
    ~HikCamera() override;
    bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
    void * handle_ = nullptr;
    std::string vid_pid_;
};

// 外观类：读配置，自动 new 出具体相机，并负责翻转
class Camera
{
public:
    Camera(const std::string & config_path);
    bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
    std::unique_ptr<CameraBase> camera_;
    int flip_code_ = 0;
};

}  // namespace io
