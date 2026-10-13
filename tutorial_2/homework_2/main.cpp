// main.cpp —— 取图 -> YOLO 识别 -> 画框 -> 显示
//
// 运行前先切到本目录（yaml 里的 assets/yolov5.xml 等是相对当前工作目录的）：
//   ./build/main
//   ./build/main -c configs/camera.yaml -y configs/yolo.yaml
//   NO_WINDOW=1 ./build/main            # 没有显示器时不开窗

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  // 命令行参数：-c 相机配置，-y yolo 配置
  std::string camera_config = "configs/camera.yaml";
  std::string yolo_config = "configs/yolo.yaml";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "-c" || arg == "--camera") && i + 1 < argc) {
      camera_config = argv[++i];
    } else if ((arg == "-y" || arg == "--yolo") && i + 1 < argc) {
      yolo_config = argv[++i];
    } else {
      std::cerr << "用法: " << argv[0] << " [-c camera.yaml] [-y yolo.yaml]\n";
      return 1;
    }
  }

  // logger 会往 logs/ 写文件，目录不存在会抛异常，先建好
  std::filesystem::create_directories("logs");

  const bool show_window = std::getenv("NO_WINDOW") == nullptr;

  try {
    // 初始化相机和 yolo
    io::Camera camera(camera_config);
    auto_aim::YOLO yolo(yolo_config, true);

    tools::logger()->info("[main] 流水线启动：camera = {}, yolo = {}", camera_config, yolo_config);

    int frame_count = 0;
    int empty_count = 0;             // 连续取图失败的次数
    const int kMaxEmptyFrames = 60;  // 连续失败这么多帧就认为设备/录像结束

    // 逐帧日志（可选）：FRAME_LOG=xxx.csv，每行是 帧号,装甲板数,最高置信度
    std::ofstream frame_log;
    if (const char * frame_log_path = std::getenv("FRAME_LOG")) {
      frame_log.open(frame_log_path);
      frame_log << "frame,count,max_confidence\n";
    }

    int fps_frames = 0;
    auto fps_begin = std::chrono::steady_clock::now();

    while (true) {
      // 取图 + 识别
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);

      if (img.empty()) {  // 空图表示取图失败
        if (++empty_count > kMaxEmptyFrames) {
          tools::logger()->warn("[main] 连续 {} 帧没有取到图，退出", empty_count);
          break;
        }
        continue;
      }
      empty_count = 0;

      const auto armors = yolo.detect(img, frame_count++);

      if (frame_log.is_open()) {
        double max_conf = 0.0;
        for (const auto & armor : armors) max_conf = std::max(max_conf, armor.confidence);
        frame_log << frame_count - 1 << ',' << armors.size() << ',' << max_conf << '\n';
      }

      // 画框
      for (const auto & armor : armors) {
        // armor.points 的顺序是：左上、左下、右下、右上
        tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(img, armor.box, cv::Scalar(0, 255, 255), 1);

        const std::string label = auto_aim::ARMOR_NAMES.at(armor.name) + " " +
                                  std::to_string(static_cast<int>(armor.confidence * 100)) + "%";
        tools::draw_text(img, label, armor.box.tl() + cv::Point(0, -5));
      }

      // 每秒打一次 fps 和装甲板数量
      ++fps_frames;
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - fps_begin).count();
      if (elapsed >= 1.0) {
        tools::logger()->info(
          "[main] fps = {:.1f}, 本帧装甲板数 = {}, 累计帧数 = {}",
          fps_frames / elapsed, armors.size(), frame_count);
        fps_frames = 0;
        fps_begin = now;
      }

      // 无窗口调试用：SAVE_RESULT=out 会把有检测结果的帧存成图片
      if (const char * save_dir = std::getenv("SAVE_RESULT")) {
        static int saved = 0;
        if (!armors.empty() && saved < 3) {
          std::filesystem::create_directories(save_dir);
          cv::imwrite(
            std::string(save_dir) + "/result_" + std::to_string(saved++) + ".jpg", img);
        }
      }

      if (show_window) {
        cv::imshow("armor", img);
        if (cv::waitKey(1) == 'q') break;
      }
    }

    tools::logger()->info("[main] 退出，共处理 {} 帧", frame_count);
  } catch (const std::exception & e) {
    const std::string what = e.what();
    tools::logger()->error("[main] 异常退出：{}", what);
    if (what.find("Camera") != std::string::npos) {
      tools::logger()->info(
        "[main] 没接相机也能离线跑整条流水线：{} -c configs/camera_replay.yaml", argv[0]);
    }
    return 1;
  }

  return 0;
}
