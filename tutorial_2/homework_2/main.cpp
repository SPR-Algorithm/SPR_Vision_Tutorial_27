#include "io/camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

#include <chrono>
#include <string>

int main(int argc, char* argv[])
{
    std::string camera_config = "configs/camera.yaml";
    std::string yolo_config   = "configs/yolo.yaml";
    for (int i = 1; i + 1 < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-c") camera_config = argv[i + 1];
        else if (arg == "-y") yolo_config = argv[i + 1];
    }

    // 初始化相机、yolo类
    io::Camera camera(camera_config);
    auto_aim::YOLO yolo(yolo_config, true);

    int frame_count = 0;
    auto last = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
        camera.read(img, timestamp);
        if (img.empty()) continue;

        // 调用yolo识别装甲板
        const auto armors = yolo.detect(img, frame_count++);

        auto now = std::chrono::steady_clock::now();
        double fps = 1.0 / std::chrono::duration<double>(now - last).count();
        last = now;

        for (const auto& armor : armors) {
            cv::Scalar draw_color = {0, 255, 0};
            if(armor.color == auto_aim::red) draw_color = {0, 0, 255};
            else if(armor.color == auto_aim::blue) draw_color = {255, 0, 0};

            tools::draw_points(img, armor.points, draw_color);

            tools::draw_text(img, auto_aim::COLORS[armor.color],
                             cv::Point(armor.points[0]), draw_color);
            
            // tools::draw_points(img, armor.points, {0, 255, 0});

        }
        tools::draw_text(img, "fps: " + std::to_string(static_cast<int>(fps)), {10, 30});
        tools::logger()->info("fps: {:.1f}, armors: {}", fps, armors.size());

        // 显示图像
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

  return 0;
}