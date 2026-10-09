#include "io/my_camera.hpp"
#include "hikrobot/include/MvCameraControl.h"
#include <iostream>
#include <unordered_map>

namespace io
{

// ---------------- HikCamera ----------------

HikCamera::HikCamera(
  const std::string & camera_name, double exposure_ms, double gain,
  const std::string & vid_pid, int flip_code)
: camera_name_(camera_name), exposure_ms_(exposure_ms), gain_(gain), vid_pid_(vid_pid),
  flip_code_(flip_code)
{
}

HikCamera::~HikCamera() { close(); }

bool HikCamera::open()
{
  MV_CC_DEVICE_INFO_LIST device_list;
  int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] EnumDevices failed: " << ret << std::endl;
    return false;
  }
  if (device_list.nDeviceNum == 0) {
    std::cerr << "[HikCamera] No camera found" << std::endl;
    return false;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] CreateHandle failed: " << ret << std::endl;
    return false;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] OpenDevice failed: " << ret << std::endl;
    return false;
  }

  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_ms_ * 1000.0));
  MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_));
  MV_CC_SetFrameRate(handle_, 60);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] StartGrabbing failed: " << ret << std::endl;
    return false;
  }

  std::cout << "[HikCamera] " << camera_name_ << " opened" << std::endl;
  return true;
}

bool HikCamera::read(cv::Mat & img)
{
  if (!handle_) return false;

  MV_FRAME_OUT raw;
  unsigned int timeout_ms = 100;
  int ret = MV_CC_GetImageBuffer(handle_, &raw, timeout_ms);
  if (ret != MV_OK) return false;

  img = transfer(&raw);
  MV_CC_FreeImageBuffer(handle_, &raw);

  if (flip_code_ != 0) cv::flip(img, img, flip_code_);
  return !img.empty();
}

void HikCamera::close()
{
  if (!handle_) return;
  MV_CC_StopGrabbing(handle_);
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
}

cv::Mat HikCamera::transfer(void * raw_ptr)
{
  auto & raw = *static_cast<MV_FRAME_OUT *>(raw_ptr);

  cv::Mat img(
    cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

  auto pixel_type = raw.stFrameInfo.enPixelType;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
    {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
    {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
    {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
    {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};

  if (type_map.count(pixel_type)) {
    cv::cvtColor(img, img, type_map.at(pixel_type));
  } else {
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;
    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    MV_CC_ConvertPixelType(handle_, &cvt_param);
  }

  return img.clone();
}

// ---------------- UsbCamera ----------------

UsbCamera::UsbCamera(int index, int flip_code) : index_(index), flip_code_(flip_code) {}

UsbCamera::~UsbCamera() { close(); }

bool UsbCamera::open()
{
  if (!cap_.open(index_)) {
    std::cerr << "[UsbCamera] Failed to open camera " << index_ << std::endl;
    return false;
  }
  std::cout << "[UsbCamera] " << index_ << " opened" << std::endl;
  return true;
}

bool UsbCamera::read(cv::Mat & img)
{
  if (!cap_.read(img) || img.empty()) return false;
  if (flip_code_ != 0) cv::flip(img, img, flip_code_);
  return true;
}

void UsbCamera::close()
{
  if (cap_.isOpened()) cap_.release();
}

// ---------------- Camera ----------------

Camera::Camera(
  const std::string & camera_name, double exposure_ms, double gain,
  const std::string & vid_pid, int flip_code, int usb_index)
{
  if (camera_name == "usb") {
    camera_ = std::make_unique<UsbCamera>(usb_index, flip_code);
  } else {
    camera_ = std::make_unique<HikCamera>(camera_name, exposure_ms, gain, vid_pid, flip_code);
  }

  if (!camera_->open()) {
    std::cerr << "[Camera] Failed to open camera" << std::endl;
    exit(1);
  }
}

Camera::~Camera() = default;

bool Camera::read(cv::Mat & img) { return camera_->read(img); }

}