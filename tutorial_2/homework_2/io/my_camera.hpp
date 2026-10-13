#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

namespace io
{

class CameraBase
{
public:
  virtual ~CameraBase() = default;

  virtual bool open() = 0;
  virtual bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual void close() = 0;
};

class HikCamera : public CameraBase
{
public:
  HikCamera(
    const std::string & camera_name, double exposure_ms, double gain,
    const std::string & vid_pid, int flip_code);
  ~HikCamera() override;

  bool open() override;
  bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  void close() override;

private:
  cv::Mat transfer(void * raw);

  std::string camera_name_;
  double exposure_ms_;
  double gain_;
  std::string vid_pid_;
  int flip_code_;
  void * handle_ = nullptr;
  bool opened_ = false;
  bool grabbing_ = false;
};

class UsbCamera : public CameraBase
{
public:
  UsbCamera(int index, int flip_code);
  ~UsbCamera() override;

  bool open() override;
  bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  void close() override;

private:
  int index_;
  int flip_code_;
  cv::VideoCapture cap_;
};

class Camera
{
public:
  explicit Camera(const std::string & config_path);
  ~Camera();

  bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io
