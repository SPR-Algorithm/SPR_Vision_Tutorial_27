#pragma once

// 第二阶段作业核心：相机类体系。
//
//   io::CameraBase   —— 抽象基类，只规定"给一张 BGR cv::Mat"这件事
//   io::HikCamera    —— 海康相机（hikrobot/include/MvCameraControl.h）
//   io::UsbCamera    —— USB 摄像头（cv::VideoCapture）
//   io::ReplayCamera —— 加分项：读一段录像文件做离线回归
//   io::Camera       —— 外观类：根据 yaml 里的 camera_name 自动挑实现，
//                       业务代码（main / tasks）只依赖这个外观。
//
// 分层：io 管硬件、tasks 管算法、tools 管工具、main 只串流水线。
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace io {

/// 抽象相机接口。任何能产出一帧图像的东西都实现这个接口。
class CameraBase {
public:
    CameraBase() = default;
    virtual ~CameraBase() = default;

    /// 打开相机。cfg 是 configs/camera.yaml 这一段节点，
    /// 各实现自己取自己关心的字段。返回 false 表示打开失败。
    virtual bool open(const YAML::Node& cfg) = 0;

    /// 关闭相机，重复调用安全。
    virtual void close() = 0;

    /// 取一帧。成功时 img 是三通道图，返回 true；
    /// 失败 / 到视频末尾返回 false，main 里据此退出循环。
    virtual bool read(cv::Mat& img) = 0;

    /// 是否已打开。
    virtual bool isOpened() const = 0;
};

/// 海康工业相机。SDK 头文件只在 .cpp 里出现，头文件不依赖 MvCameraControl.h。
class HikCamera : public CameraBase {
public:
    HikCamera();
    ~HikCamera() override;

    bool open(const YAML::Node& cfg) override;
    void close() override;
    bool read(cv::Mat& img) override;
    bool isOpened() const override;

private:
    // 用 PIMPL 把海康句柄藏在 .cpp 里（加分项 2：tasks 层不依赖海康 SDK）
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// USB 摄像头。直接用 cv::VideoCapture。
class UsbCamera : public CameraBase {
public:
    UsbCamera();
    ~UsbCamera() override;

    bool open(const YAML::Node& cfg) override;
    void close() override;
    bool read(cv::Mat& img) override;
    bool isOpened() const override;

private:
    cv::VideoCapture cap_;
};

/// 加分项 1：离线回放。cv::VideoCapture 读录像文件，读完返回 false。
class ReplayCamera : public CameraBase {
public:
    ReplayCamera();
    ~ReplayCamera() override;

    bool open(const YAML::Node& cfg) override;
    void close() override;
    bool read(cv::Mat& img) override;
    bool isOpened() const override;

private:
    cv::VideoCapture cap_;
};

/// 外观类。main 里只 new 这个：
///   io::Camera camera(tools::load("configs/camera.yaml"));
///   while (camera.read(img)) { ... }
/// 它根据 yaml 的 camera_name 挑 hikrobot / usb / replay，
/// 并统一按 flip_code 翻转图像。
class Camera {
public:
    explicit Camera(const YAML::Node& cfg);
    ~Camera();

    bool read(cv::Mat& img);
    bool isOpened() const;
    void close();

private:
    std::unique_ptr<CameraBase> impl_;
    int flip_code_ = 1;
};

}  // namespace io