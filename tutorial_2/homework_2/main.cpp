#include <chrono>
#include <exception>
#include <iostream>
#include <string>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>

#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

int main(int argc, char** argv)
{
  // ① 解析命令行参数：不给参数就用 configs/ 下的默认值
  std::string camera_config = "configs/camera.yaml";
  std::string yolo_config   = "configs/yolo.yaml";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-c" && i + 1 < argc) {
      camera_config = argv[++i];
    } else if (arg == "-y" && i + 1 < argc) {
      yolo_config = argv[++i];
    } else {
      std::cerr << "用法: " << argv[0] << " [-c camera.yaml] [-y yolo.yaml]\n";
      return 1;
    }
  }

  try {
    // ② 初始化：相机 + YOLO
    io::Camera camera(camera_config);
    if (!camera.open()) {
      tools::logger()->error("相机打开失败：请检查设备是否连接、camera_name 是否正确");
      return 1;
    }
    auto_aim::YOLO yolo(yolo_config, false);
    tools::logger()->info("初始化完成，进入主循环");

    int frame_count = 0;        // 总帧数
    int frames_since_log = 0;   // 距上次打日志的帧数
    int empty_count = 0;        // 连续取不到图的次数
    auto last_time = std::chrono::steady_clock::now();

    // ③ 主循环
    while (true) {
      cv::Mat img = camera.read();
      if (img.empty()) {
        if (++empty_count > 30) {          // 连续 30 帧没图 → 认为结束了
          tools::logger()->info("取图结束，退出");
          break;
        }
        tools::logger()->warn("取图失败，跳过这一帧");
        continue;
      }
      empty_count = 0;

      if (frame_count % 30 == 0) {
        tools::logger()->info(
          "第 {} 帧: {}x{} 平均亮度={:.1f}", frame_count, img.cols, img.rows, cv::mean(img)[0]);
      }

      auto armors = yolo.detect(img, frame_count);

      // ④ 画四个角点 + 名字/置信度
      for (const auto & armor : armors) {
        tools::draw_points(img, armor.points, {0, 255, 0});
        auto info = fmt::format("{} {:.2f}",
                                auto_aim::ARMOR_NAMES[armor.name], armor.confidence);
        tools::draw_text(img, info, armor.center, {0, 255, 0});
      }

      // ⑤ fps：每满 1 秒打印一次
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - last_time).count();
      if (elapsed >= 1.0) {
        double fps = frames_since_log / elapsed;
        tools::logger()->info("fps: {:.1f}, armors: {}", fps, armors.size());
        frames_since_log = 0;
        last_time = now;
      }

      // ⑥ 显示 + 按 q 退出
      try {
        cv::imshow("img", img);
      } catch (const std::exception & e) {
        tools::logger()->error("imshow 打不开窗口: {}", e.what());
        return 1;
      }
      int key = cv::waitKey(1) & 0xFF;
      if (key == 'q') break;

      ++frame_count;
      ++frames_since_log;
    }
  } catch (const std::exception & e) {
    tools::logger()->error("初始化失败: {}", e.what());
    return 1;
  }

  return 0;
}
