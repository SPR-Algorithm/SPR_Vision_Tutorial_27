#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

class CameraBase {
public:
    virtual ~CameraBase() = default;
    virtual bool open() = 0;
    virtual bool getFrame(cv::Mat& frame) = 0;
    virtual void close() = 0;
    virtual std::string name() const = 0; };

class HikCamera : public CameraBase {
public:
    HikCamera(int exposure_ms, double gain, int flip_code);
    ~HikCamera() override;

    bool open() override;
    bool getFrame(cv::Mat& frame) override;
    void close() override;
    std::string name() const override
{ return "HikCamera"; }
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    int exposure_ms_;
    double gain_;
    int flip_code_;
};

class UsbCamera : public CameraBase {
public :
    UsbCamera(int index, int exposure_ms, double gain, int flip_code);
    ~UsbCamera() override;

    bool open() override;
    bool getFrame(cv::Mat& frame) override;
    void close() override;
    std::string name() const override
{ return "UsbCamera"; }

private:
    cv::VideoCapture cap_;
    int index_;
    int exposure_ms_;
    double gain_;
    int flip_code_;
};

class Camera {
public:
    Camera(std::unique_ptr<CameraBase> impl) : impl_(std::move(impl)) {}
    bool open() { return impl_->open(); }
    bool getFrame(cv::Mat& frame) { return
impl_->getFrame(frame); }
    void close() { impl_->close(); }
    std::string name() const { return impl_->name(); }
private:
    std::unique_ptr<CameraBase> impl_;
};
