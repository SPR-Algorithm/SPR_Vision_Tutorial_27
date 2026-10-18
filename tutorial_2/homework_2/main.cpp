#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

#include <chrono>

int main()
{
    // 1. 初始化相机和 yolo
    io::Camera camera("configs/camera.yaml");
    auto_aim::YOLO yolo("configs/yolo.yaml");

    // fps 统计用
    int frame_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;

        // 2. 调用相机读取一帧图像
        if (!camera.read(img, timestamp)) {
            continue;
        }

        // 3. 调用 yolo 识别装甲板
        std::list<auto_aim::Armor> armors = yolo.detect(img);

        // 4. 画出识别结果：框 + 四个角点 + 颜色标签
        for (const auto_aim::Armor & armor : armors) {
            cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);
            tools::draw_points(img, armor.points, cv::Scalar(0, 0, 255), 2);
            tools::draw_text(
                img, auto_aim::COLORS[armor.color], armor.box.tl() + cv::Point(0, -5));
        }

        // 5. 每秒打印一次 fps 和装甲板数量
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_start).count();
        if (elapsed >= 1.0) {
            double fps = frame_count / elapsed;
            tools::logger()->info("fps: {:.2f}, armor: {}", fps, armors.size());
            frame_count = 0;
            fps_start = now;
        }

        // 6. 显示图像
        cv::resize(img, img, cv::Size(640, 480));
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
