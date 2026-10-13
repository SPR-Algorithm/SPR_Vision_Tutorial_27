#include "io/my_camera.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>

#include "hikrobot/include/MvCameraControl.h"
#include "tools/yaml.hpp"

namespace io
{
namespace
{

bool parse_vid_pid(const std::string & text, std::uint16_t & vid, std::uint16_t & pid)
{
  auto pos = text.find(':');
  if (pos == std::string::npos) return false;
  try {
    auto v = std::stoul(text.substr(0, pos), nullptr, 16);
    auto p = std::stoul(text.substr(pos + 1), nullptr, 16);
    if (v > 0xffff || p > 0xffff) return false;
    vid = static_cast<std::uint16_t>(v);
    pid = static_cast<std::uint16_t>(p);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

std::size_t row_step(const MV_FRAME_OUT_INFO_EX & info, std::size_t min_step)
{
  const auto h = static_cast<std::size_t>(info.nHeight);
  if (h == 0) return min_step;
  if (info.nFrameLen < min_step * h) return min_step;
  if (info.nFrameLen % h != 0) return min_step;
  return info.nFrameLen / h;
}

}  // namespace

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
  close();

  MV_CC_DEVICE_INFO_LIST device_list{};
  int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] EnumDevices failed: " << ret << std::endl;
    return false;
  }
  if (device_list.nDeviceNum == 0) {
    std::cerr << "[HikCamera] No camera found" << std::endl;
    return false;
  }

  MV_CC_DEVICE_INFO * selected = device_list.pDeviceInfo[0];
  std::uint16_t vid = 0;
  std::uint16_t pid = 0;
  if (parse_vid_pid(vid_pid_, vid, pid)) {
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      auto * info = device_list.pDeviceInfo[i];
      if ((info->nTLayerType & MV_USB_DEVICE) != 0 &&
          info->SpecialInfo.stUsb3VInfo.idVendor == vid &&
          info->SpecialInfo.stUsb3VInfo.idProduct == pid) {
        selected = info;
        break;
      }
    }
  }

  ret = MV_CC_CreateHandle(&handle_, selected);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] CreateHandle failed: " << ret << std::endl;
    handle_ = nullptr;
    return false;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] OpenDevice failed: " << ret << std::endl;
    close();
    return false;
  }
  opened_ = true;

  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_ms_ * 1000.0));
  MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_));
  // 有些相机上次被设成了触发模式，必须关掉，否则 GetImageBuffer 永远超时
  MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    std::cerr << "[HikCamera] StartGrabbing failed: " << ret << std::endl;
    close();
    return false;
  }
  grabbing_ = true;

  std::cout << "[HikCamera] " << camera_name_ << " opened" << std::endl;
  return true;
}

bool HikCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  if (handle_ == nullptr) return false;

  MV_FRAME_OUT raw{};
  const int ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
  if (ret != MV_OK) return false;

  img = transfer(&raw);
  MV_CC_FreeImageBuffer(handle_, &raw);

  if (img.empty()) return false;
  if (flip_code_ != 0) cv::flip(img, img, flip_code_);

  timestamp = std::chrono::steady_clock::now();
  return true;
}

void HikCamera::close()
{
  if (handle_ == nullptr) return;

  if (grabbing_) {
    MV_CC_StopGrabbing(handle_);
    grabbing_ = false;
  }
  if (opened_) {
    MV_CC_CloseDevice(handle_);
    opened_ = false;
  }
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
}

cv::Mat HikCamera::transfer(void * raw_ptr)
{
  auto & raw = *static_cast<MV_FRAME_OUT *>(raw_ptr);
  const int w = static_cast<int>(raw.stFrameInfo.nWidth);
  const int h = static_cast<int>(raw.stFrameInfo.nHeight);
  if (w <= 0 || h <= 0 || raw.pBufAddr == nullptr) return {};

  const auto pixel_type = raw.stFrameInfo.enPixelType;
  cv::Mat bgr;

  // 常见 8bit 格式直接用 OpenCV 转，统一输出 BGR
  switch (pixel_type) {
    case PixelType_Gvsp_BGR8_Packed: {
      const auto step = row_step(raw.stFrameInfo, static_cast<std::size_t>(w) * 3);
      bgr = cv::Mat(h, w, CV_8UC3, raw.pBufAddr, step).clone();
      break;
    }
    case PixelType_Gvsp_RGB8_Packed: {
      const auto step = row_step(raw.stFrameInfo, static_cast<std::size_t>(w) * 3);
      cv::cvtColor(cv::Mat(h, w, CV_8UC3, raw.pBufAddr, step), bgr, cv::COLOR_RGB2BGR);
      break;
    }
    case PixelType_Gvsp_Mono8: {
      const auto step = row_step(raw.stFrameInfo, static_cast<std::size_t>(w));
      cv::cvtColor(cv::Mat(h, w, CV_8UC1, raw.pBufAddr, step), bgr, cv::COLOR_GRAY2BGR);
      break;
    }
    // 注意 OpenCV 的 BayerXX2BGR 名字取的是第二行第二/三列，和 Hik 的
    // BayerRG8/GR8/GB8/BG8 命名要左右对调，否则 R/B 会反（人脸/手会发蓝）。
    case PixelType_Gvsp_BayerRG8: {
      const auto step = row_step(raw.stFrameInfo, static_cast<std::size_t>(w));
      cv::cvtColor(cv::Mat(h, w, CV_8UC1, raw.pBufAddr, step), bgr, cv::COLOR_BayerBG2BGR);
      break;
    }
    case PixelType_Gvsp_BayerGR8: {
      const auto step = row_step(raw.stFrameInfo, static_cast<std::size_t>(w));
      cv::cvtColor(cv::Mat(h, w, CV_8UC1, raw.pBufAddr, step), bgr, cv::COLOR_BayerGB2BGR);
      break;
    }
    case PixelType_Gvsp_BayerGB8: {
      const auto step = row_step(raw.stFrameInfo, static_cast<std::size_t>(w));
      cv::cvtColor(cv::Mat(h, w, CV_8UC1, raw.pBufAddr, step), bgr, cv::COLOR_BayerGR2BGR);
      break;
    }
    case PixelType_Gvsp_BayerBG8: {
      const auto step = row_step(raw.stFrameInfo, static_cast<std::size_t>(w));
      cv::cvtColor(cv::Mat(h, w, CV_8UC1, raw.pBufAddr, step), bgr, cv::COLOR_BayerRG2BGR);
      break;
    }
    default: {
      // 10/12bit Bayer、YUV 等交给 SDK 转 BGR8
      MV_CC_PIXEL_CONVERT_PARAM cvt{};
      cvt.nWidth = static_cast<unsigned short>(w);
      cvt.nHeight = static_cast<unsigned short>(h);
      cvt.enSrcPixelType = pixel_type;
      cvt.pSrcData = raw.pBufAddr;
      cvt.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

      bgr.create(h, w, CV_8UC3);
      cvt.pDstBuffer = bgr.data;
      cvt.nDstBufferSize = static_cast<unsigned int>(bgr.total() * bgr.elemSize());

      const int ret = MV_CC_ConvertPixelType(handle_, &cvt);
      if (ret != MV_OK) {
        std::cerr << "[HikCamera] ConvertPixelType failed: " << ret
                  << ", pixel_type=" << static_cast<int>(pixel_type) << std::endl;
        return {};
      }
      break;
    }
  }

  return bgr;
}

UsbCamera::UsbCamera(int index, int flip_code) : index_(index), flip_code_(flip_code) {}

UsbCamera::~UsbCamera() { close(); }

bool UsbCamera::open()
{
  bool ok = cap_.open(index_, cv::CAP_V4L2);
  if (!ok) ok = cap_.open(index_);
  if (!ok) {
    std::cerr << "[UsbCamera] Failed to open camera " << index_ << std::endl;
    return false;
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap_.set(cv::CAP_PROP_FPS, 30);
  // buffersize=1 时 VMware USB 直通会大量丢帧/坏帧，画面上表现为拉伸、发糊；
  // 4 个缓冲可以稳定取流。
  cap_.set(cv::CAP_PROP_BUFFERSIZE, 4);
  // 很多 UVC 摄像头（尤其内置摄像头）默认 YUYV 在当前分辨率下带宽不够，
  // 表现为 open 成功但 read 一直 select() timeout，这里优先请求 MJPG。
  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  std::cout << "[UsbCamera] " << index_ << " opened: "
            << cap_.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << cap_.get(cv::CAP_PROP_FRAME_HEIGHT)
            << " fourcc=0x" << std::hex << static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC))
            << std::dec << std::endl;
  return true;
}

bool UsbCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  if (!cap_.isOpened()) return false;
  if (!cap_.read(img) || img.empty()) return false;
  if (flip_code_ != 0) cv::flip(img, img, flip_code_);
  timestamp = std::chrono::steady_clock::now();
  return true;
}

void UsbCamera::close()
{
  if (cap_.isOpened()) cap_.release();
}

Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  auto camera_name = tools::read<std::string>(yaml, "camera_name", "hikrobot");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms", 10.0);
  auto gain = tools::read<double>(yaml, "gain", 20.0);
  auto vid_pid = tools::read<std::string>(yaml, "vid_pid", "");
  auto flip_code = tools::read<int>(yaml, "flip_code", 0);
  auto usb_index = tools::read<int>(yaml, "usb_index", 0);

  if (camera_name == "usb") {
    camera_ = std::make_unique<UsbCamera>(usb_index, flip_code);
  } else if (camera_name == "hikrobot" || camera_name == "hik") {
    camera_ = std::make_unique<HikCamera>(camera_name, exposure_ms, gain, vid_pid, flip_code);
  } else {
    std::cerr << "[Camera] Unknown camera_name: " << camera_name << std::endl;
    exit(1);
  }

  if (!camera_->open()) {
    std::cerr << "[Camera] Failed to open camera: " << camera_name << std::endl;
    exit(1);
  }
}

Camera::~Camera() = default;

bool Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  return camera_->read(img, timestamp);
}

}  // namespace io
