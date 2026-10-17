#include "my_camera.hpp"

#include <chrono>
#include <stdexcept>
#include <unordered_map>

#include "MvCameraControl.h"
#include "tools/yaml.hpp"

namespace io
{

namespace
{

cv::Mat transfer(MV_FRAME_OUT & raw)
{
  MV_CC_PIXEL_CONVERT_PARAM cvt_param;
  cv::Mat img(
    cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U,
    raw.pBufAddr);

  cvt_param.nWidth = raw.stFrameInfo.nWidth;
  cvt_param.nHeight = raw.stFrameInfo.nHeight;
  cvt_param.pSrcData = raw.pBufAddr;
  cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
  cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
  cvt_param.pDstBuffer = img.data;
  cvt_param.nDstBufferSize = img.total() * img.elemSize();
  cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

  auto pixel_type = raw.stFrameInfo.enPixelType;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes>
    type_map = {
      {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
      {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
      {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
      {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};

  auto it = type_map.find(pixel_type);
  if (it != type_map.end()) {
    cv::cvtColor(img, img, it->second);
  }

  return img;
}

}  

HikCamera::HikCamera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  auto gain = tools::read<double>(yaml, "gain");

  MV_CC_DEVICE_INFO_LIST device_list;
  int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK || device_list.nDeviceNum == 0) {
    throw std::runtime_error("HikCamera: no device found");
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    throw std::runtime_error("HikCamera: create handle failed");
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    throw std::runtime_error("HikCamera: open device failed");
  }

  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_ms * 1000.0);
  MV_CC_SetFloatValue(handle_, "Gain", gain);
  MV_CC_SetFrameRate(handle_, 60);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    throw std::runtime_error("HikCamera: start grabbing failed");
  }
}

HikCamera::~HikCamera()
{
  if (handle_) {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
  }
}

void HikCamera::read(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  MV_FRAME_OUT raw;
  unsigned int nMsec = 100;
  int ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
  if (ret != MV_OK) {
    img = cv::Mat();
    return;
  }

  img = transfer(raw);
  timestamp = std::chrono::steady_clock::now();

  MV_CC_FreeImageBuffer(handle_, &raw);
}

UsbCamera::UsbCamera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  int device_id = 0;

  cap_.open(device_id);
  if (!cap_.isOpened()) {
    throw std::runtime_error("UsbCamera: cannot open camera");
  }
}

UsbCamera::~UsbCamera()
{
  if (cap_.isOpened()) cap_.release();
}

void UsbCamera::read(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  if (!cap_.read(img) || img.empty()) {
    img = cv::Mat();
    return;
  }
  timestamp = std::chrono::steady_clock::now();
}

Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto name = tools::read<std::string>(yaml, "camera_name");
  flip_code_ = tools::read<int>(yaml, "flip_code", 2);

  if (name == "hikrobot") {
    impl_ = std::make_unique<HikCamera>(config_path);
  } else if (name == "usb") {
    impl_ = std::make_unique<UsbCamera>(config_path);
  } else {
    throw std::runtime_error("Camera: unknown camera_name: " + name);
  }
}

void Camera::read(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  impl_->read(img, timestamp);
  if (img.empty()) return;
  if (flip_code_ >= 0) {
    cv::flip(img, img, flip_code_);
  }
}

}  




