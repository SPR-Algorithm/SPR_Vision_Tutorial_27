#pragma once


#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
    virtual ~CameraBase() = default;
    virtual void read(
        cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

class HikCamera : public CameraBase
{
public:
    HikCamera(double exposure_ms, double gain, const std::string & vid_pid);
    ~HikCamera() override;
    void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
    void close();
    void * handle_ = nullptr;
};

class UsbCamera : public CameraBase
{
public:
    explicit UsbCamera(const std::string & vid_pid);

    ~UsbCamera() override;
    void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
private:
    cv::VideoCapture cap_;
};

class ReplayCamera : public CameraBase
{
public:
    explicit ReplayCamera(const std::string & video_path);

    ~ReplayCamera() override;
    void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
private:
    cv::VideoCapture cap_;
    int max_frame_count_ = 0;
    int frame_count_ = 0;
};

class Camera
{
public:
    explicit Camera(const std::string & config_path);
    void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
private:
    std::unique_ptr<CameraBase> camera_;    
    int flip_code_ = -1;

};
}