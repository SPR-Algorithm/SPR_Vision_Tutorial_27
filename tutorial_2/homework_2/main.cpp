#include "io/my_camera.hpp"

#include <chrono>
#include <string>

#include <fmt/format.h>
#include <opencv2/opencv.hpp>

#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  std::string camera_config = "configs/camera.yaml";
  std::string yolo_config = "configs/yolo.yaml";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-c" || arg == "--camera") && i + 1 < argc) {
      camera_config = argv[++i];
    } else if ((arg == "-y" || arg == "--yolo") && i + 1 < argc) {
      yolo_config = argv[++i];
    }
  }

  io::Camera camera(camera_config);
  auto_aim::YOLO yolo(yolo_config);

  int frame_count = 0;
  int frames = 0;
  double fps = 0.0;
  auto last = std::chrono::steady_clock::now();

  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    if (!camera.read(img, timestamp)) {
      cv::waitKey(1);
      continue;
    }

    auto armors = yolo.detect(img, frame_count++);

    for (const auto & armor : armors) {
      tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
      tools::draw_text(
        img, fmt::format("{:.2f}", armor.confidence), armor.center, cv::Scalar(0, 255, 0), 0.6, 2);
    }

    ++frames;
    auto now = std::chrono::steady_clock::now();
    if (now - last >= std::chrono::seconds(1)) {
      fps = frames / std::chrono::duration<double>(now - last).count();
      tools::logger()->info("fps: {:.1f}, armors: {}", fps, armors.size());
      last = now;
      frames = 0;
    }

    tools::draw_text(
      img, fmt::format("fps: {:.1f}  armors: {}", fps, armors.size()), cv::Point(10, 30),
      cv::Scalar(0, 255, 255), 0.8, 2);

    cv::imshow("img", img);
    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
  }

  cv::destroyAllWindows();
  return 0;
}
