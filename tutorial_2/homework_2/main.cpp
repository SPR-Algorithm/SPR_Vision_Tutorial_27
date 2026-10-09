#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

#include <string>

int main(int argc, char ** argv)
{
  std::string camera_config = "configs/camera.yaml";
  std::string yolo_config = "configs/yolo.yaml";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-c" && i + 1 < argc) {
      camera_config = argv[++i];
    } else if (arg == "-y" && i + 1 < argc) {
      yolo_config = argv[++i];
    }
  }

  auto cam_yaml = tools::load(camera_config);
  auto camera_name = tools::read<std::string>(cam_yaml, "camera_name", "hik");
  auto exposure_ms = tools::read<double>(cam_yaml, "exposure_ms", 10.0);
  auto gain = tools::read<double>(cam_yaml, "gain", 20.0);
  auto vid_pid = tools::read<std::string>(cam_yaml, "vid_pid", "");
  auto flip_code = tools::read<int>(cam_yaml, "flip_code", 0);
  auto usb_index = tools::read<int>(cam_yaml, "usb_index", 0);

  io::Camera camera(camera_name, exposure_ms, gain, vid_pid, flip_code, usb_index);
  auto_aim::YOLO yolo(yolo_config);

  cv::Mat img;
  int frame_count = 0;
  double fps = 0.0;
  int64 t0 = cv::getTickCount();

  while (true) {
    if (!camera.read(img)) continue;
    if (img.empty()) break;

    auto armors = yolo.detect(img, frame_count++);

    for (const auto & armor : armors) {
      if (armor.points.size() == 4) {
        tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0), 2);
      }
    }

    int64 t1 = cv::getTickCount();
    fps = cv::getTickFrequency() / (t1 - t0);
    t0 = t1;

    tools::logger()->info("fps: {:.1f}, armors: {}", fps, armors.size());

    tools::draw_text(
      img, cv::format("fps: %.1f", fps), cv::Point(10, 30), cv::Scalar(0, 255, 255), 1.0, 2);
    tools::draw_text(
      img, cv::format("armors: %zu", armors.size()), cv::Point(10, 60),
      cv::Scalar(0, 255, 255), 1.0, 2);

    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}