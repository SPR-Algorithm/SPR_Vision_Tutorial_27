#include <iostream>
#include "io/my_camera.hpp"
#include "tools/yaml.hpp"
#include "tools/logger.hpp"
#include "tools/img_tools.hpp"
#include "tasks/yolo.hpp"

int main(int argc, char** argv) {
    std::string cam_cfg_path = "configs/camera.yaml";
    std::string yolo_cfg_path = "configs/yolo.yaml";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-c" && i + 1 < argc) cam_cfg_path = argv[++i];
        else if (arg == "-y" && i + 1 < argc) yolo_cfg_path = argv[++i];
    }
    auto cam_yaml = tools::load(cam_cfg_path);
    std::string camera_name = tools::read<std::string>(cam_yaml, "camera_name");
    int exposure_ms = tools::read<int>(cam_yaml, "exposure_ms");
    double gain = tools::read<double>(cam_yaml, "gain");
    int flip_code = tools::read<int>(cam_yaml, "flip_code");

    std::unique_ptr<CameraBase> cam_impl;
    if (camera_name == "hik") {
        cam_impl = std::make_unique<HikCamera>(exposure_ms, gain, flip_code);
    } else if (camera_name == "usb") {
            int index = tools::read<int>(cam_yaml, "vid_pid");
            cam_impl = std::make_unique<UsbCamera>(index, exposure_ms, gain, flip_code);
    } else {
        SPDLOG_ERROR("未知的相机类型: {}", camera_name);
        return -1;
    }

    Camera camera(std::move(cam_impl));
        std::cout << "准备打开相机" << std::endl;

    if (!camera.open()) {
            SPDLOG_ERROR("相机打开失败!");
            return -1;
        }
        std::cout <<"相机已打开,准备加载YOLO" << std::endl;


    auto_aim::YOLO yolo(yolo_cfg_path);
        std::cout << "YOLO加载成功,准备取图" << std::endl;

    cv::Mat frame;
    auto last_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    double fps = 0.0;

    while (true) {
        if (!camera.getFrame(frame) || frame.empty()) continue;
        std::cout << "成功获取画面,准备识别" << std::endl;
        auto armors = yolo.detect(frame);
                std::cout << "识别完成,装甲板数量：" << armors.size() << std::endl;
        for (const auto& armor : armors) {
            std::vector<cv::Point> int_points;
            for (const auto& p : armor.points) {
                int_points.push_back(cv::Point(cvRound(p.x), cvRound(p.y)));
            }
            cv::polylines(frame, int_points, true, {0, 255, 0}, 2);
            for (const auto& p : int_points) {
                cv::circle(frame, p, 4, {0, 0, 255}, -1);
            }
            cv::putText(frame, "Armor", int_points[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
        }

        frame_count++;
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - last_time).count();
        if (elapsed_time >= 1.0) {
            fps = frame_count / elapsed_time;
            frame_count = 0;
            last_time = current_time;
        }
        SPDLOG_INFO("FPS: {:.2f}, Armor Count: {}", fps, armors.size());
        cv::imshow("Camera Test", frame);
        if (cv::waitKey(1) == 'q') break;
    }
    return 0;
}

    