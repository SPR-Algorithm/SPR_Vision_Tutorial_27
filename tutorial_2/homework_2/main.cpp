#include <chrono>
#include <list>
#include <string>

#include <opencv2/opencv.hpp>

#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ? |                     | 打印帮助信息 }"
    "{camera c       | configs/camera.yaml | 相机配置文件的路径 }"
    "{yolo y         | configs/yolo.yaml   | YOLO 配置文件的路径 }";

  cv::CommandLineParser cli(argc, argv, keys);

  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto camera_config = cli.get<std::string>("camera");
  auto yolo_config = cli.get<std::string>("yolo");

  // 初始化相机、yolo类
  io::Camera camera("hik");
  //io::Camera camera("usb");
  //io::Camera camera("replay");
  auto_aim::YOLO yolo(yolo_config, true);

  int frame_count = 0;
  std::chrono::steady_clock::time_point timestamp;

  while (true) { 
    cv::Mat img;

    camera.read(img, timestamp);

    if (img.empty()) {continue;}

    // 调用yolo识别装甲板
    auto begin = std::chrono::steady_clock::now();
    const std::list<auto_aim::Armor> armors = yolo.detect(img, frame_count++);

    // 把识别结果画到图上
    for (const auto & armor : armors) {
      if (armor.points.empty()) continue;
      tools::draw_points(img, armor.points, {0, 255, 0});
      tools::draw_text(
        img, auto_aim::ARMOR_NAMES[armor.name] + " " + auto_aim::COLORS[armor.color],
        cv::Point(armor.points[0]));
    }

    auto dt =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    tools::logger()->info("{:.2f} fps, {} armors", 1 / dt, armors.size());

    // 显示图像
    cv::imshow("img", img);
    if (cv::waitKey(30) == 'q') {
      break;
    }
  }

  return 0;
}