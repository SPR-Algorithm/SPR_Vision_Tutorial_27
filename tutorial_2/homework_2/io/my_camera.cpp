#include "io/my_camera.hpp"

#include <stdexcept>
#include <unordered_map>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io {
Camera::Camera(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  // TODO 添加 相机翻转功能
  flip_code_ = tools::read<int>(yaml, "flip_code", 2);
  if (flip_code_ != 0 && flip_code_ != 1 && flip_code_ != -1 &&
      flip_code_ != 2) {
    throw std::runtime_error("Invalid flip_code: " +
                             std::to_string(flip_code_));
  }

  if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikCamera>(exposure_ms, gain, vid_pid);
  }

  // TODO: 添加 mindvision 相机支持（需先实现 MindVision 类）
  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat &img,
                  std::chrono::steady_clock::time_point &timestamp) {
  camera_->read(img, timestamp);
  if (flip_code_ != 2) {
    cv::flip(img, img, flip_code_);
  }
}

// ---------------------------------------------------------------------------
// HikCamera：海康工业相机的实现
// ---------------------------------------------------------------------------

HikCamera::HikCamera(double exposure_ms, double gain,
                     const std::string &vid_pid)
    : exposure_us_(exposure_ms * 1e3), gain_(gain) {
  auto index = vid_pid.find(':');
  if (index != std::string::npos) {
    try {
      vid_ = std::stoi(vid_pid.substr(0, index), nullptr, 16);
      pid_ = std::stoi(vid_pid.substr(index + 1), nullptr, 16);
    } catch (const std::exception &) {
      tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
      vid_ = -1;
      pid_ = -1;
    }
  }

  open();
}

HikCamera::~HikCamera() { close(); }

void HikCamera::read(cv::Mat &img,
                     std::chrono::steady_clock::time_point &timestamp) {
  if (handle_ == nullptr) {
    tools::logger()->warn("HikCamera is not opened, skip read()!");
    return;
  }

  MV_FRAME_OUT raw;
  const unsigned int timeout_ms = 100;

  auto ret = MV_CC_GetImageBuffer(handle_, &raw, timeout_ms);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x}", ret);
    return;
  }

  // 先转换再释放：转换结果是独立内存，不再依赖 raw 的缓冲区
  img = convert(raw);
  timestamp = std::chrono::steady_clock::now();

  ret = MV_CC_FreeImageBuffer(handle_, &raw);
  if (ret != MV_OK)
    tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
}

void HikCamera::open() {
  MV_CC_DEVICE_INFO_LIST device_list;
  auto ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  auto *device = pick_device(device_list, vid_, pid_);

  ret = MV_CC_CreateHandle(&handle_, device);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    handle_ = nullptr;
    return;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    close();
    return;
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  MV_CC_SetFrameRate(handle_, 60);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    close();
    return;
  }

  tools::logger()->info("HikCamera opened.");
}

void HikCamera::close() {
  if (handle_ == nullptr)
    return;

  auto ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK)
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK)
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK)
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);

  handle_ = nullptr;
}

void HikCamera::set_float_value(const std::string &name, double value) {
  auto ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name,
                          value, ret);
  }
}

void HikCamera::set_enum_value(const std::string &name, unsigned int value) {
  auto ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name,
                          value, ret);
  }
}

MV_CC_DEVICE_INFO *HikCamera::pick_device(MV_CC_DEVICE_INFO_LIST &device_list,
                                          int vid, int pid) {
  if (vid != -1 && pid != -1) {
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      auto *info = device_list.pDeviceInfo[i];
      if (info->nTLayerType != MV_USB_DEVICE)
        continue;
      if (info->SpecialInfo.stUsb3VInfo.idVendor != vid)
        continue;
      if (info->SpecialInfo.stUsb3VInfo.idProduct != pid)
        continue;
      return info;
    }
    tools::logger()->warn(
        "Not found camera with vid_pid: {:04x}:{:04x}, use default.", vid, pid);
  }

  return device_list.pDeviceInfo[0];
}

cv::Mat HikCamera::convert(const MV_FRAME_OUT &raw) {
  const auto &info = raw.stFrameInfo;
  cv::Mat src(info.nHeight, info.nWidth, CV_8U, raw.pBufAddr);

  // 首次取帧时打印一次实际像素格式，方便排查 Bayer 排列 / 通道顺序问题
  static bool pixel_type_logged = false;
  if (!pixel_type_logged) {
    tools::logger()->info("pixel type: {:#x}",
                          static_cast<unsigned int>(info.enPixelType));
    pixel_type_logged = true;
  }

  // 注意：OpenCV 的 BayerXX 字母取自「第二行第 2、3 个像素」，而海康 / GenICam
  // 的命名用「首行前两个像素」，两套命名正好错开一格（RG<->BG、GR<->GB）。
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes>
      bayer_map = {{PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGB2BGR},
                   {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerBG2BGR},
                   {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGR2BGR},
                   {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerRG2BGR}};

  auto it = bayer_map.find(info.enPixelType);
  if (it != bayer_map.end()) {
    cv::Mat bgr;
    cv::cvtColor(src, bgr, it->second);
    return bgr;
  }

  if (info.enPixelType == PixelType_Gvsp_Mono8) {
    cv::Mat bgr;
    cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }

  tools::logger()->warn("Unsupported pixel type: {:#x}, return raw image.",
                        info.enPixelType);
  return src.clone();
}

} // namespace io