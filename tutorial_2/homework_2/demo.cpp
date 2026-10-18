#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

#include <chrono>
#include <string>

// 演示用：读摄像头 -> YOLO 识别 -> 画框画角点 -> 存图（不弹窗，headless 也能跑）
int main()
{
    io::Camera camera("configs/camera.yaml");
    auto_aim::YOLO yolo("configs/yolo.yaml");

    for (int i = 0; i < 10; ++i) {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;

        if (!camera.read(img, timestamp)) {
            tools::logger()->warn("第 {} 帧读取失败", i);
            continue;
        }

        auto armors = yolo.detect(img);

        for (const auto & armor : armors) {
            cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);
            tools::draw_points(img, armor.points, cv::Scalar(0, 0, 255), 2);
            tools::draw_text(
                img, auto_aim::COLORS[armor.color], armor.box.tl() + cv::Point(0, -5));
        }

        tools::logger()->info("第 {} 帧识别到 {} 个装甲板", i, armors.size());

        std::string file = "demo_" + std::to_string(i) + ".png";
        cv::imwrite(file, img);
    }

    return 0;
}
