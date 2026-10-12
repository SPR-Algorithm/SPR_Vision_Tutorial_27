#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"
#include "tools/fps_counter.hpp"

int main() {
    // ---- 1. 初始化相机、yolo 类 ----
    auto camera_cfg = tools::load("configs/camera.yaml");
    io::Camera camera(camera_cfg);
    if (!camera.isOpened()) {
        tools::logger()->error("camera not opened, exit");
        return 1;
    }

    auto_aim::YOLO yolo("configs/yolo.yaml");

    tools::FpsCounter fps;
    int frame_count = 0;

    // ---- 2. 主循环 ----
    while (true) {
        // 2.1 调用相机读取图像
        cv::Mat img;
        if (!camera.read(img)) {
            tools::logger()->info("camera read failed, exit");
            break;
        }

        // 2.2 调用 yolo 识别装甲板
        auto armors = yolo.detect(img, frame_count++);

        // 2.3 在图上画四个角点 + 装甲板 id
        for (const auto& armor : armors) {
            tools::draw_points(img, armor.points);
            if (!armor.points.empty()) {
                std::string text = tools::COLORS[armor.color] + " " +
                                   tools::ARMOR_NAMES[armor.name];
                tools::draw_text(img, text,
                                cv::Point(static_cast<int>(armor.points[0].x),
                                          static_cast<int>(armor.points[0].y) - 5));
            }
        }

        // 2.4 显示图像（按骨架要求 resize 到 640x480）
        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("img", img);

        // 2.5 日志：fps + 装甲板数量
        fps.tick();
        tools::logger()->info("fps={:.1f}  armors={}", fps.fps(), armors.size());

        // 2.6 按 q 退出
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    camera.close();
    return 0;
}