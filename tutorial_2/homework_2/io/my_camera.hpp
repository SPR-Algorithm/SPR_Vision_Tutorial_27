#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

namespace io
{

class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual bool open() = 0;
  virtual bool read(cv::Mat & img) = 0;
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
  bool read(cv::Mat & img) override;
  void close() override;

private:
  cv::Mat transfer(void * raw);

  std::string camera_name_;
  double exposure_ms_;
  double gain_;
  std::string vid_pid_;
  int flip_code_;
  void * handle_ = nullptr;
};

class UsbCamera : public CameraBase
{
public:
  UsbCamera(int index, int flip_code);
  ~UsbCamera() override;

  bool open() override;
  bool read(cv::Mat & img) override;
  void close() override;

private:
  int index_;
  int flip_code_;
  cv::VideoCapture cap_;
};

class Camera
{
public:
  Camera(
    const std::string & camera_name, double exposure_ms, double gain,
    const std::string & vid_pid, int flip_code, int usb_index = 0);
  ~Camera();

  bool read(cv::Mat & img);

private:
  std::unique_ptr<CameraBase> camera_;
};

}