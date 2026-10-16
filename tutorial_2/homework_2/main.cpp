#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
  std::string camera_config = "configs/camera.yaml";
  std::string yolo_config = "configs/yolo.yaml";
  
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];

    if ((arg == "-c" || arg == "--camera") && i + 1 < argc)
    {
      camera_config = argv[++i];
    }
    else if ((arg == "-y" || arg == "--yolo") && i + 1 < argc)
    {
      yolo_config = argv[++i];
    }
    else
    {
      std::cerr << "Usage: " << argv[0]
                << " [-c camera.yaml] [-y yolo.yaml]\n";
      return 1;
    }
  }

  std::filesystem::create_directories("logs");
  const bool show_window = std::getenv("NO_WINDOW") == nullptr;

  try
  {
    // TODO 1: 初始化相机、yolo类

    io::Camera camera(camera_config);
    auto_aim::YOLO yolo(yolo_config, true);

    int yolo_frame_id = 0;
    int total_frames = 0;
    int empty_count = 0;
    const int kMaxEmptyFrames = 60;
    int fps_frames = 0;
    auto fps_begin = std::chrono::steady_clock::now();

    while (true)
    {
      // while (1) {
      // 调用相机读取图像
      cv::Mat img;
      io::TimePoint timestamp;
      camera.read(img, timestamp);
      if (img.empty())
      {
        if (++empty_count > kMaxEmptyFrames)
        {
          tools::logger()->warn("[main] No frame, exit");
          break;
        }
        continue;
      }
      empty_count = 0;

      // TODO 2: 调用yolo识别装甲板
      const auto armors = yolo.detect(img, yolo_frame_id++);
      ++total_frames;

      for (const auto &armor : armors)
      {
        tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(img, armor.box, cv::Scalar(0, 255, 255), 1);
        const std::string label = auto_aim::ARMOR_NAMES.at(armor.name) + " " + std::to_string(static_cast<int>(armor.confidence * 100)) + "%";
        tools::draw_text(img, label, armor.box.tl() + cv::Point(0, -5));
      }

      ++fps_frames;
      const auto now = std::chrono::steady_clock::now();
      const double elapsed =
          std::chrono::duration<double>(now - fps_begin).count();
      if (elapsed >= 1.0)
      {
        tools::logger()->info(
            "[main] fps={:.1f}, armors={}, total_frames={}",
            fps_frames / elapsed, armors.size(), total_frames);
        fps_frames = 0;
        fps_begin = now;
      }

      // TODO 3: 显示图像
      if (show_window)
      {
        cv::imshow("armor", img);
        if (cv::waitKey(1) == 'q')
        {
          tools::logger()->info("[main] User pressed 'q'");
          break;
        }
      }
    }
    if (show_window)
    {
      cv::destroyAllWindows();
    }

    tools::logger()->info("[main] Exit, processed {} frames", total_frames);
  }
  catch (const std::exception &e)
  {
    tools::logger()->error("[main] Exception: {}", e.what());
    return 1;
  }
  return 0;
}
