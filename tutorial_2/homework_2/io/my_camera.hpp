#pragma once
#include <iostream>
#include <string>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>

namespace io
{
 class CameraBase
{

public:
  virtual ~CameraBase() = default;
  virtual void read(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

// HikCamera
class HikCamera : public CameraBase
{
public:
  explicit HikCamera(const std::string & config_path);
  ~HikCamera() override;

  void read(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  void * handle_ = nullptr;
};

// USB 
class UsbCamera : public CameraBase
{
public:
  explicit UsbCamera(const std::string & config_path);
  ~UsbCamera() override;

  void read(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  cv::VideoCapture cap_;
};

// Camera
class Camera
{
public:
  explicit Camera(const std::string & config_path);

  void read(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> impl_;
  int flip_code_ = -1;
};

}  