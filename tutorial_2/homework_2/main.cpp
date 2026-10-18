#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include <iostream>
#include <string>
#include "tools/yaml.hpp"

int main() {
    auto config = tools::load("configs/camera.yaml");
    
    io::Camera camera;
    if (!camera.init("configs/camera.yaml")) {
        tools::logger()->error("相机初始化失败！");
        return -1;
    }

    auto_aim::YOLO yolo("configs/yolo.yaml");

    cv::Mat img;
    double timestamp;
    double fps = 0;
    double last_time = cv::getTickCount() / cv::getTickFrequency();
    int frame_count = 0;

    tools::logger()->info("流水线启动，按 q 退出...");

    while (true) {
        if (!camera.read(img, timestamp)) {
            tools::logger()->error("读取图像失败！");
            break;
        }
        if (img.empty()) continue;

        std::list<auto_aim::Armor> armors = yolo.detect(img, frame_count);

        for (const auto& armor : armors) {
            if (armor.points.size() == 4) {
                cv::line(img, armor.points[0], armor.points[1], cv::Scalar(0, 255, 0), 2);
                cv::line(img, armor.points[1], armor.points[2], cv::Scalar(0, 255, 0), 2);
                cv::line(img, armor.points[2], armor.points[3], cv::Scalar(0, 255, 0), 2);
                cv::line(img, armor.points[3], armor.points[0], cv::Scalar(0, 255, 0), 2);
                cv::circle(img, (armor.points[0] + armor.points[2]) / 2, 5, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("img", img);

        frame_count++;
        double current_time = cv::getTickCount() / cv::getTickFrequency();
        if (current_time - last_time >= 1.0) {
            fps = frame_count / (current_time - last_time);
            tools::logger()->info("FPS: {:.2f}, 装甲板数量: {}", fps, armors.size());
            frame_count = 0;
            last_time = current_time;
        }

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}