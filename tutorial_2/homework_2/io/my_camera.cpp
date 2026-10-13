#include "my_camera.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "hikrobot/include/MvCameraControl.h"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{

// 海康 SDK 只给错误码，出问题时不好排查，这里做个可读化
std::string sdk_error_text(int ret)
{
  switch (static_cast<unsigned int>(ret)) {
    case MV_OK: return "成功";
    case MV_E_HANDLE: return "错误或无效的句柄";
    case MV_E_SUPPORT: return "不支持的功能";
    case MV_E_BUFOVER: return "缓存已满";
    case MV_E_CALLORDER: return "函数调用顺序错误";
    case MV_E_PARAMETER: return "参数错误";
    case MV_E_RESOURCE: return "资源申请失败";
    case MV_E_NODATA: return "无数据";
    case MV_E_PRECONDITION: return "前置条件有误，或运行环境已发生变化";
    case MV_E_NOENOUGH_BUF: return "传入的内存空间不足";
    case MV_E_LOAD_LIBRARY: return "动态库加载失败";
    case MV_E_ACCESS_DENIED: return "设备无访问权限（可能需要 sudo 或 udev 规则）";
    case MV_E_BUSY: return "设备忙（可能被 MVS 等其他程序占用）";
    case MV_E_USB_READ: return "读 USB 出错";
    case MV_E_USB_WRITE: return "写 USB 出错";
    case MV_E_USB_DEVICE: return "USB 设备异常";
    case MV_E_USB_DRIVER: return "USB 驱动不匹配或未安装";
    case MV_E_USB_UNKNOW: return "USB 未知错误";
    case MV_E_UNKNOW: return "未知错误";
    default: return "未映射的错误码";
  }
}

// 格式化成 "0x80000006 (资源申请失败)"
std::string sdk_error_str(int ret)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0')
      << static_cast<unsigned int>(ret) << std::dec << " (" << sdk_error_text(ret) << ")";
  return oss.str();
}

// "2bdf:0001" -> {0x2bdf, 0x0001}；解析失败返回 {-1, -1}，表示不按 vid_pid 筛选
std::pair<int, int> parse_vid_pid(const std::string & vid_pid)
{
  const auto pos = vid_pid.find(':');
  if (pos == std::string::npos) return {-1, -1};
  try {
    const int vid = std::stoi(vid_pid.substr(0, pos), nullptr, 16);
    const int pid = std::stoi(vid_pid.substr(pos + 1), nullptr, 16);
    return {vid, pid};
  } catch (const std::exception &) {
    return {-1, -1};
  }
}

// 按 vid:pid 在 /sys/class/video4linux 里定位对应的 /dev/videoN
// 匹配不到返回 0，保证只有一路摄像头时也能用
int find_usb_index(const std::string & vid_pid)
{
  const auto [target_vid, target_pid] = parse_vid_pid(vid_pid);
  if (target_vid < 0) return 0;

  std::error_code ec;
  const std::filesystem::path root("/sys/class/video4linux");
  if (!std::filesystem::exists(root, ec)) return 0;

  for (const auto & entry : std::filesystem::directory_iterator(root, ec)) {
    const std::string name = entry.path().filename().string();  // 形如 video0
    if (name.rfind("video", 0) != 0) continue;

    const auto dev = entry.path() / "device";
    auto read_hex = [&dev](const char * file) -> int {
      std::ifstream in((dev / file).string());
      std::string value;
      if (!(in >> value)) return -1;
      try {
        return std::stoi(value, nullptr, 16);
      } catch (const std::exception &) {
        return -1;
      }
    };

    if (read_hex("idVendor") == target_vid && read_hex("idProduct") == target_pid) {
      return std::stoi(name.substr(5));
    }
  }
  return 0;
}

// 用 SDK 把原始帧转成 BGR
// 转换结果写在 cv::Mat 自己申请的内存里，和 SDK 缓冲无关
cv::Mat frame_to_bgr(void * handle, MV_FRAME_OUT & raw)
{
  cv::Mat bgr(raw.stFrameInfo.nHeight, raw.stFrameInfo.nWidth, CV_8UC3);

  MV_CC_PIXEL_CONVERT_PARAM cvt = {};
  cvt.nWidth = static_cast<unsigned short>(raw.stFrameInfo.nWidth);
  cvt.nHeight = static_cast<unsigned short>(raw.stFrameInfo.nHeight);
  cvt.pSrcData = static_cast<unsigned char *>(raw.pBufAddr);
  cvt.nSrcDataLen = raw.stFrameInfo.nFrameLen;
  cvt.enSrcPixelType = raw.stFrameInfo.enPixelType;
  cvt.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  cvt.pDstBuffer = bgr.data;
  cvt.nDstBufferSize = static_cast<unsigned int>(bgr.total() * bgr.elemSize());

  if (MV_CC_ConvertPixelType(handle, &cvt) != MV_OK) return cv::Mat{};
  return bgr;
}

}  // namespace

// =============================================================================
// HikCamera
// =============================================================================
HikCamera::HikCamera(double exposure_ms, double gain, const std::string & vid_pid)
{
  // 1) 枚举设备
  MV_CC_DEVICE_INFO_LIST device_list = {};
  const int enum_ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (enum_ret != MV_OK) {
    // 枚举这一步失败通常不是相机本身的问题，而是运行环境：
    // 没插、没直通到虚拟机、驱动或动态库没装好、权限不够
    throw std::runtime_error(
      "[HikCamera] MV_CC_EnumDevices 失败：" + sdk_error_str(enum_ret) +
      "\n  排查：1) 相机是否已插好并上电；"
      "\n        2) 如果跑在虚拟机里，USB 设备是否已从宿主机直通进来；"
      "\n        3) 终端执行 lsusb，看能不能找到海康的 VID（一般为 2bdf）；"
      "\n        4) 是否装了 MVS / 驱动，或进程是否被其他程序占用。");
  }
  if (device_list.nDeviceNum == 0) {
    // 枚举成功但一个都没扫到，和上面是两种不同的情况，分开报
    throw std::runtime_error(
      "[HikCamera] SDK 枚举成功，但没找到任何海康相机（nDeviceNum = 0）。"
      "\n  排查：相机是否已插好并上电；虚拟机里是否已 USB 直通。");
  }

  // 按 vid_pid 挑设备，挑不到就用第 0 个
  int index = 0;
  const auto [target_vid, target_pid] = parse_vid_pid(vid_pid);
  if (target_vid >= 0) {
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      const auto * info = device_list.pDeviceInfo[i];
      if (info->nTLayerType != MV_USB_DEVICE) continue;
      const auto & usb = info->SpecialInfo.stUsb3VInfo;
      if (usb.idVendor == target_vid && usb.idProduct == target_pid) {
        index = static_cast<int>(i);
        break;
      }
    }
  }

  // 2) 创建句柄 -> 打开设备 -> 设置参数 -> 开始取流
  const int create_ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[index]);
  if (create_ret != MV_OK) {
    handle_ = nullptr;
    throw std::runtime_error("[HikCamera] 创建句柄失败：" + sdk_error_str(create_ret));
  }
  const int open_ret = MV_CC_OpenDevice(handle_);
  if (open_ret != MV_OK) {
    close();  // 构造函数抛异常时析构不会被调用，已经拿到的资源要在这里还回去
    throw std::runtime_error(
      "[HikCamera] 打开设备失败：" + sdk_error_str(open_ret) +
      "\n  提示：MV_E_ACCESS_DENIED 一般是权限问题；"
      "MV_E_BUSY 一般是相机被 MVS 等程序占用了。");
  }

  MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_ms * 1000.0));
  MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain));
  MV_CC_SetFrameRate(handle_, 60);

  const int grab_ret = MV_CC_StartGrabbing(handle_);
  if (grab_ret != MV_OK) {
    close();
    throw std::runtime_error("[HikCamera] 开始取流失败：" + sdk_error_str(grab_ret));
  }

  tools::logger()->info(
    "[HikCamera] 打开成功：设备 #{}, 曝光 {}ms, 增益 {}", index, exposure_ms, gain);
}

HikCamera::~HikCamera()
{
  close();
}

void HikCamera::close()
{
  if (handle_ == nullptr) return;
  MV_CC_StopGrabbing(handle_);  // 没在取流时调用也无害
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
}

void HikCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  img = cv::Mat{};  // 取图失败就留空图，调用方用 img.empty() 判断

  MV_FRAME_OUT raw = {};
  if (MV_CC_GetImageBuffer(handle_, &raw, 100) != MV_OK) return;  // 超时或丢帧

  img = frame_to_bgr(handle_, raw);

  // 必须转换完再还缓冲，否则就是在读一块已经还给 SDK 的内存
  MV_CC_FreeImageBuffer(handle_, &raw);

  if (!img.empty()) timestamp = std::chrono::steady_clock::now();
}

// =============================================================================
// UsbCamera
// =============================================================================
UsbCamera::UsbCamera(const std::string & vid_pid)
{
  // 先试 vid_pid 定位到的编号，不行再把 /dev/video0~5 都试一遍。
  // 笔记本上红外摄像头常常是 video0、彩色是 video2，写死 0 容易打不开。
  const int resolved = find_usb_index(vid_pid);
  std::vector<int> candidates{resolved};
  for (int i = 0; i <= 5; ++i) {
    if (i != resolved) candidates.push_back(i);
  }

  for (const int index : candidates) {
    if (cap_.open(index)) {
      tools::logger()->info("[UsbCamera] 打开成功：/dev/video{}", index);
      return;
    }
  }

  throw std::runtime_error(
    "[UsbCamera] /dev/video0~5 一个都打不开。"
    "\n  排查：1) 终端执行 ls /dev/video*，看系统有没有摄像头设备；"
    "\n        2) 跑在虚拟机里的话，摄像头是否已从宿主机直通进来；"
    "\n        3) 摄像头是否被 Zoom / Teams / cheese 等程序占用；"
    "\n        4) 当前用户是否在 video 组里（groups 看一下）。");
}

UsbCamera::~UsbCamera()
{
  if (cap_.isOpened()) cap_.release();
}

void UsbCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  img = cv::Mat{};
  if (!cap_.read(img)) {
    img = cv::Mat{};
    return;
  }
  timestamp = std::chrono::steady_clock::now();
}

// =============================================================================
// ReplayCamera
// =============================================================================
ReplayCamera::ReplayCamera(const std::string & video_path) : cap_(video_path)
{
  if (!cap_.isOpened()) {
    throw std::runtime_error("[ReplayCamera] 打不开录像文件：" + video_path);
  }
  // 总帧数从视频里读出来，放完就结束
  max_frame_count_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_COUNT));
  tools::logger()->info(
    "[ReplayCamera] 打开成功：{}（共 {} 帧）", video_path, max_frame_count_);
}

ReplayCamera::~ReplayCamera()
{
  if (cap_.isOpened()) cap_.release();
}

void ReplayCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  img = cv::Mat{};

  // 放完就交空图，表示结束
  if (max_frame_count_ > 0 && frame_count_ >= max_frame_count_) return;

  if (!cap_.read(img)) {
    img = cv::Mat{};
    return;
  }
  ++frame_count_;
  timestamp = std::chrono::steady_clock::now();
}

// =============================================================================
// Camera
// =============================================================================
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  const auto name = tools::read<std::string>(yaml, "camera_name");
  const auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  const auto gain = tools::read<double>(yaml, "gain");
  const auto vid_pid = tools::read<std::string>(yaml, "vid_pid", std::string{});
  flip_code_ = tools::read<int>(yaml, "flip_code", -1);

  // 按配置创建对应的相机，这里是唯一出现具体相机名字的地方
  if (name == "hikrobot") {
    camera_ = std::make_unique<HikCamera>(exposure_ms, gain, vid_pid);
  } else if (name == "usb") {
    camera_ = std::make_unique<UsbCamera>(vid_pid);
  } else if (name == "replay") {
    const auto path = tools::read<std::string>(yaml, "replay_path");
    camera_ = std::make_unique<ReplayCamera>(path);
  } else {
    throw std::runtime_error("[Camera] 不认识的 camera_name: " + name);
  }

  tools::logger()->info("[Camera] {} 初始化完成，flip_code = {}", name, flip_code_);
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);

  // 相机装反时按配置翻转
  if (!img.empty() && flip_code_ >= 0) {
    cv::flip(img, img, flip_code_);
  }
}

}  // namespace io
