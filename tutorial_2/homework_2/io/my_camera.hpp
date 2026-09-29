#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "MvCameraControl.h"

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

class HikCamera : public CameraBase
{
public:
  // exposure_ms: 曝光时间（毫秒），内部换算成 SDK 需要的微秒
  // vid_pid:     形如 "2bdf:0001" 的 USB VID:PID；留空则用枚举到的第一台设备
  explicit HikCamera(double exposure_ms, double gain, const std::string & vid_pid = "");
  ~HikCamera() override;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

  bool is_opened() const { return handle_ != nullptr; }

private:
  void open();
  void close();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  // 从枚举结果中按 VID/PID 挑选设备；找不到时返回列表中的第一台
  static MV_CC_DEVICE_INFO * pick_device(MV_CC_DEVICE_INFO_LIST & device_list, int vid, int pid);

  // 把原始帧转换成 OpenCV 常用的 BGR 图像（不依赖 raw 的生命周期）
  static cv::Mat convert(const MV_FRAME_OUT & raw);

  void * handle_ = nullptr;
  double exposure_us_ = 0.0;
  double gain_ = 0.0;
  int vid_ = -1;
  int pid_ = -1;
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> camera_;
  int flip_code_ = 2; // opencv flip code: 0 vertical, 1 horizontal, -1 both 2 default no flip
};

}  // namespace io

#endif  // IO__CAMERA_HPP