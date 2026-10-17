#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "hikrobot/include/MvCameraControl.h"

namespace io
{
using TimePoint = std::chrono::system_clock::time_point;

// 抽象基类：所有相机只要实现 open / read，就能接入流水线
class CameraBase
{
public:
  explicit CameraBase(const YAML::Node & yaml);
  virtual ~CameraBase() = default;

  CameraBase(const CameraBase &) = delete;
  CameraBase & operator=(const CameraBase &) = delete;

  virtual bool open() = 0;

  // 读出一张 BGR 的 cv::Mat；失败（掉帧）返回 false，img 为空
  virtual bool read(cv::Mat & img) = 0;

  TimePoint timestamp() const;

protected:
  YAML::Node yaml_;
  TimePoint timestamp_;
};

// 海康工业相机：把 example.cpp 里面向过程的取图流程封成类
class HikCamera : public CameraBase
{
public:
  explicit HikCamera(const YAML::Node & yaml);
  ~HikCamera() override;

  bool open() override;
  bool read(cv::Mat & img) override;

private:
  void * handle_ = nullptr;
  bool opened_ = false;
  bool grabbing_ = false;

  double exposure_ms_;
  double gain_;
  double frame_rate_;
  std::string vid_pid_;
};

// USB 摄像头：用 cv::VideoCapture 读本机摄像头
class UsbCamera : public CameraBase
{
public:
  explicit UsbCamera(const YAML::Node & yaml);
  ~UsbCamera() override = default;

  bool open() override;
  bool read(cv::Mat & img) override;

private:
  cv::VideoCapture capture_;
  std::string fourcc_;
  int usb_width_;
  int usb_height_;
  int usb_fps_;
  int device_id_;
  double exposure_ms_;
  double gain_;
};

// 外观类：读配置、按 camera_name 造具体相机、统一做 flip 翻转
class Camera
{
public:
  explicit Camera(const std::string & config_path);
  ~Camera() = default;

  Camera(const Camera &) = delete;
  Camera & operator=(const Camera &) = delete;

  // 取图（BGR），并按配置的 flip_code 翻转
  bool read(cv::Mat & img);

  TimePoint timestamp() const;

private:
  std::unique_ptr<CameraBase> camera_;
  int flip_code_;
};

}  // namespace io
