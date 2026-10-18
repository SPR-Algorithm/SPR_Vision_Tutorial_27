#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

#include <chrono>
#include <filesystem>
#include <string>

// 演示：读视频 -> YOLO 识别装甲板 -> 画框画角点 -> 把识别到的帧存成图（headless 可跑）
// 用法：./build/video_demo [视频路径] [最多处理帧数，0=全部]
int main(int argc, char ** argv)
{
    std::string video_path =
        "/home/rsve/Desktop/SPR_Vision_Tutorial_27/tutorial_2/example/opencv/demo/demo.mp4";
    int max_frames = 0;  // 0 表示跑完整个视频

    if (argc > 1) video_path = argv[1];
    if (argc > 2) max_frames = std::stoi(argv[2]);

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        tools::logger()->error("打不开视频: {}", video_path);
        return 1;
    }

    auto_aim::YOLO yolo("configs/yolo.yaml");

    // 识别到装甲板的帧会存到这里，方便在 VS Code 里点开看
    std::filesystem::create_directories("video_out");

    int fps_frame = 0;
    int total_frames = 0;
    int saved_count = 0;
    auto fps_start = std::chrono::steady_clock::now();

    while (true) {
        cv::Mat img;
        cap >> img;
        if (img.empty()) break;  // 视频放完

        auto armors = yolo.detect(img);

        // 画框 + 四个角点 + 颜色标签（和 main.cpp 一样）
        for (const auto & armor : armors) {
            cv::rectangle(img, armor.box, cv::Scalar(0, 255, 0), 2);
            tools::draw_points(img, armor.points, cv::Scalar(0, 0, 255), 2);
            tools::draw_text(
                img, auto_aim::COLORS[armor.color], armor.box.tl() + cv::Point(0, -5));
        }

        fps_frame++;
        total_frames++;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_start).count();
        if (elapsed >= 1.0) {
            tools::logger()->info("fps: {:.2f}, 当前帧 {} 个装甲板, 已处理 {} 帧",
                                  fps_frame / elapsed, armors.size(), total_frames);
            fps_frame = 0;
            fps_start = now;
        }

        // 识别到的帧存一张图，最多存 30 张
        if (!armors.empty() && saved_count < 30) {
            std::string file = "video_out/frame_" + std::to_string(saved_count) + ".jpg";
            cv::imwrite(file, img);
            saved_count++;
        }

        if (max_frames > 0 && total_frames >= max_frames) break;
    }

    tools::logger()->info("完成，共处理 {} 帧，保存 {} 张识别结果图到 video_out/", total_frames, saved_count);
    return 0;
}
