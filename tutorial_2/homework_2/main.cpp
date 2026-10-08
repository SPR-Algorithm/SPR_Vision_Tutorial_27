#include "io/camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

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
  io::Camera camera(camera_config);
  auto_aim::YOLO yolo(yolo_config, true);

  int frame_count = 0;
  std::chrono::steady_clock::time_point timestamp;

  while (true) {
    cv::Mat img;
    camera.read(img, timestamp);
    if (img.empty()) continue;

  // 调用yolo识别装甲板

    // 显示图像
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') {
      break;
    }
  }

  return 0;
}