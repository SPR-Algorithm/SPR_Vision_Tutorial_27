#include <algorithm>
#include <chrono>
#include <filesystem>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/my_camera.hpp"
#include "tasks/armor.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace
{
// 把 Armor 转成 "颜色-编号-大小" 的标签
std::string armor_label(const auto_aim::Armor & armor)
{
  const auto & color = auto_aim::COLORS[static_cast<std::size_t>(armor.color)];
  const auto & name = auto_aim::ARMOR_NAMES[static_cast<std::size_t>(armor.name)];
  const auto & type = auto_aim::ARMOR_TYPES[static_cast<std::size_t>(armor.type)];
  return color + "-" + name + "-" + type;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string camera_config = "configs/camera.yaml";
  std::string yolo_config = "configs/yolo.yaml";
  for (int i = 1; i < argc - 1; ++i) {
    const std::string arg = argv[i];
    if (arg == "-c") camera_config = argv[++i];
    else if (arg == "-y") yolo_config = argv[++i];
  }

  // ---------- 初始化相机、yolo 类 ----------
  std::filesystem::create_directories("logs");

  io::Camera camera(camera_config);
  auto_aim::YOLO yolo(yolo_config, false);

  tools::logger()->info("[Main] Camera and YOLO initialized.");

  // ---------- fps 统计 ----------
  const int window = 30;
  std::vector<double> frame_times;
  frame_times.reserve(window);
  auto last_time = std::chrono::steady_clock::now();

  int frame_count = 0;
  while (true) {
    // 调用相机读取图像
    cv::Mat img;
    if (!camera.read(img)) {
      tools::logger()->warn("[Main] Empty img, camera drop!");
      if (cv::waitKey(1) == 'q') break;
      continue;
    }
    ++frame_count;

    // 调用yolo识别装甲板
    const auto armors = yolo.detect(img, frame_count);

    // ---------- 画框：四个角点 + 中心 + 标签 ----------
    for (const auto & armor : armors) {
      const cv::Scalar color = armor.color == auto_aim::red ? cv::Scalar(0, 0, 255)
                                                            : cv::Scalar(255, 128, 0);
      tools::draw_points(img, armor.points, color);  // 左上、左下、右下、右上
      tools::draw_point(img, armor.center, color);
      if (armor.box.width > 0) {
        tools::draw_text(img, armor_label(armor), {armor.box.x, armor.box.y - 10}, color, 0.8);
      }
    }

    // ---------- 日志：fps + 装甲板数量 ----------
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_time).count();
    last_time = now;
    if (dt > 0) frame_times.push_back(dt);
    if (static_cast<int>(frame_times.size()) > window) frame_times.erase(frame_times.begin());
    const double avg = std::accumulate(frame_times.begin(), frame_times.end(), 0.0) / frame_times.size();
    const double fps = avg > 0 ? 1.0 / avg : 0.0;
    tools::logger()->info(
      "[Main] fps: {:.1f} | armors: {} | frame_count: {}", fps, armors.size(), frame_count);

    // ---------- 显示图像 ----------
    cv::resize(img, img, cv::Size(640, 480));
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;  // 按 q 退出
  }

  return 0;
}
