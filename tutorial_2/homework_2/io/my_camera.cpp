#include "my_camera.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
CameraBase::CameraBase(const YAML::Node & yaml) : yaml_(yaml) {}

TimePoint CameraBase::timestamp() const { return timestamp_; }

// ==================== HikCamera ====================

// "2bdf:0001" -> vid / pid
static bool parse_vid_pid(const std::string & str, unsigned int & vid, unsigned int & pid)
{
  return std::sscanf(str.c_str(), "%x:%x", &vid, &pid) == 2;
}

static void to_bgr(MV_FRAME_OUT & raw, cv::Mat & img)
{
  const cv::Size size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight);
  const auto pixel_type = raw.stFrameInfo.enPixelType;

  if (pixel_type == PixelType_Gvsp_BGR8_Packed) {
    cv::Mat bgr(size, CV_8UC3, raw.pBufAddr);
    img = bgr.clone();
    return;
  }

  cv::Mat src(size, CV_8UC1, raw.pBufAddr);
  switch (pixel_type) {
    case PixelType_Gvsp_Mono8:
      cv::cvtColor(src, img, cv::COLOR_GRAY2BGR);
      break;
    case PixelType_Gvsp_BayerGR8:
      cv::cvtColor(src, img, cv::COLOR_BayerGR2BGR);
      break;
    case PixelType_Gvsp_BayerRG8:
      cv::cvtColor(src, img, cv::COLOR_BayerRG2BGR);
      break;
    case PixelType_Gvsp_BayerGB8:
      cv::cvtColor(src, img, cv::COLOR_BayerGB2BGR);
      break;
    case PixelType_Gvsp_BayerBG8:
      cv::cvtColor(src, img, cv::COLOR_BayerBG2BGR);
      break;
    default:
      tools::logger()->error(
        "[Camera] Unsupported pixel type: 0x{:x}!", static_cast<int>(pixel_type));
      img.release();
      break;
  }
}

HikCamera::HikCamera(const YAML::Node & yaml) : CameraBase(yaml)
{
  exposure_ms_ = tools::read<double>(yaml_, "exposure_ms");
  gain_ = tools::read<double>(yaml_, "gain");
  vid_pid_ = tools::read<std::string>(yaml_, "vid_pid");
  frame_rate_ = tools::read<double>(yaml_, "frame_rate", 60.0);
}

HikCamera::~HikCamera()
{
  if (handle_ != nullptr) {
    if (grabbing_) MV_CC_StopGrabbing(handle_);
    if (opened_) MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
}

bool HikCamera::open()
{
  unsigned int vid = 0, pid = 0;
  if (!parse_vid_pid(vid_pid_, vid, pid)) {
    tools::logger()->error("[Camera] Invalid vid_pid: {}!", vid_pid_);
    return false;
  }

  MV_CC_DEVICE_INFO_LIST device_list = {};
  int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->error("[Camera] EnumDevices failed: {}!", ret);
    return false;
  }

  std::size_t index = 0;
  bool found = false;
  for (std::size_t i = 0; i < device_list.nDeviceNum; ++i) {
    const auto & info = *device_list.pDeviceInfo[i];
    if (info.nTLayerType & MV_USB_DEVICE) {
      const auto & usb = info.SpecialInfo.stUsb3VInfo;
      if (usb.idVendor == vid && usb.idProduct == pid) {
        index = i;
        found = true;
        break;
      }
    }
  }
  if (!found) {
    tools::logger()->error(
      "[Camera] Device {} not found! ({} devices online)", vid_pid_, device_list.nDeviceNum);
    return false;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[index]);
  if (ret != MV_OK) {
    tools::logger()->error("[Camera] CreateHandle failed: {}!", ret);
    return false;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->error("[Camera] OpenDevice failed: {}!", ret);
    return false;
  }
  opened_ = true;

  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  if (MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_ms_ * 1000)) !=
      MV_OK)
    tools::logger()->warn("[Camera] Set ExposureTime failed!");  // µs
  if (MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_)) != MV_OK)
    tools::logger()->warn("[Camera] Set Gain failed!");
  if (MV_CC_SetFrameRate(handle_, static_cast<float>(frame_rate_)) != MV_OK)
    tools::logger()->warn("[Camera] Set FrameRate failed!");

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->error("[Camera] StartGrabbing failed: {}!", ret);
    return false;
  }
  grabbing_ = true;

  tools::logger()->info("[Camera] HikCamera {} opened.", vid_pid_);
  return true;
}

bool HikCamera::read(cv::Mat & img)
{
  MV_FRAME_OUT raw;
  const int ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
  if (ret != MV_OK) {
    img.release();
    return false;
  }

  to_bgr(raw, img);
  MV_CC_FreeImageBuffer(handle_, &raw);

  timestamp_ = std::chrono::system_clock::now();
  return !img.empty();
}

// ==================== UsbCamera ====================

UsbCamera::UsbCamera(const YAML::Node & yaml) : CameraBase(yaml)
{
  device_id_ = tools::read<int>(yaml_, "device_id", 0);
  fourcc_ = tools::read<std::string>(yaml_, "usb_fourcc", "MJPG");
  usb_width_ = tools::read<int>(yaml_, "usb_width", 640);
  usb_height_ = tools::read<int>(yaml_, "usb_height", 480);
  usb_fps_ = tools::read<int>(yaml_, "usb_fps", 60);
  exposure_ms_ = tools::read<double>(yaml_, "exposure_ms", 10.0);
  gain_ = tools::read<double>(yaml_, "gain", 0.0);
}

bool UsbCamera::open()
{
  if (!capture_.open(device_id_)) {
    tools::logger()->error("[Camera] Failed to open USB camera #{}!", device_id_);
    return false;
  }
  capture_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
  capture_.set(cv::CAP_PROP_FRAME_WIDTH,  usb_width_);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, usb_height_);
  capture_.set(cv::CAP_PROP_FPS, usb_fps_);
  capture_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  capture_.set(cv::CAP_PROP_EXPOSURE, exposure_ms_);
  capture_.set(cv::CAP_PROP_GAIN, gain_);

  tools::logger()->info("[Camera] UsbCamera #{} opened. {}x{} fourcc={} fps={}", device_id_, usb_width_, usb_height_, fourcc_, usb_fps_);
  return true;
}

bool UsbCamera::read(cv::Mat & img)
{
  if (!capture_.read(img) || img.empty()) {
    img.release();
    return false;
  }
  timestamp_ = std::chrono::system_clock::now();
  return true;
}

// ==================== Camera（外观类） ====================

Camera::Camera(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  const auto camera_name = tools::read<std::string>(yaml, "camera_name");
  flip_code_ = tools::read<int>(yaml, "flip_code");

  if (camera_name == "hikrobot") {
    camera_ = std::make_unique<HikCamera>(yaml);
  } else if (camera_name == "usb") {
    camera_ = std::make_unique<UsbCamera>(yaml);
  } else {
    tools::logger()->error("[Camera] Unknown camera_name: {}!", camera_name);
    exit(1);
  }

  if (!camera_->open()) {
    tools::logger()->error("[Camera] Failed to open camera: {}!", camera_name);
    exit(1);
  }
}

bool Camera::read(cv::Mat & img)
{
  if (!camera_->read(img)) return false;

  // flip_code: 0 不翻转 / 1 水平 / 2 旋转 180° / 3 垂直（也兼容 OpenCV 原生 -1）
  if (flip_code_ == 1) {
    cv::flip(img, img, 1);
  } else if (flip_code_ == 2 || flip_code_ == -1) {
    cv::flip(img, img, -1);
  } else if (flip_code_ == 3) {
    cv::flip(img, img, 0);
  }
  return true;
}

TimePoint Camera::timestamp() const { return camera_->timestamp(); }

}  // namespace io
