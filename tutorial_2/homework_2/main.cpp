#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/yaml.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
    std::string camera_config = "configs/camera.yaml";
    std::string yolo_config = "configs/yolo.yaml";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-c" && i + 1 < argc) {
            camera_config = argv[++i];
        } else if (arg == "-y" && i + 1 < argc) {
            yolo_config = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0]
                      << " [-c camera.yaml] [-y yolo.yaml]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument or missing value: " << arg << '\n';
            std::cerr << "Usage: " << argv[0]
                      << " [-c camera.yaml] [-y yolo.yaml]\n";
            return 1;
        }
    }

    try {
        const YAML::Node config = tools::load(camera_config);
        const int flip_code = tools::read<int>(config, "flip_code", -1);

        io::Camera camera(camera_config);
        auto_aim::YOLO yolo(yolo_config);

        cv::Mat img;
        int frame_count = 0;
        double fps = 0.0;

        auto last_time = std::chrono::steady_clock::now();

        std::cout << "Camera config: " << camera_config << '\n';
        std::cout << "YOLO config: " << yolo_config << '\n';
        std::cout << "Press 'q' or ESC to quit.\n";

        while (true) {
            if (!camera.read(img) || img.empty()) {
                std::cerr << "[main] Failed to read camera frame.\n";
                break;
            }

            if (flip_code >= -1) {
                cv::Mat flipped;
                cv::flip(img, flipped, flip_code);
                img = std::move(flipped);
            }

            const auto armors = yolo.detect(img, frame_count);

            for (const auto & armor : armors) {
                tools::draw_points(img, armor.points, {0, 255, 0}, 2);
            }

            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - last_time).count();

            if (elapsed > 0.0) {
                fps = 1.0 / elapsed;
            }
            last_time = now;

            tools::draw_text(
                img, cv::format("FPS: %.1f", fps),
                cv::Point(20, 35), {0, 255, 255}, 0.8, 2);

            tools::draw_text(
                img, cv::format("Armors: %d", static_cast<int>(armors.size())),
                cv::Point(20, 70), {0, 255, 255}, 0.8, 2);

            cv::imshow("YOLO Detection", img);

            ++frame_count;

            const int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q' || key == 27) {
                break;
            }
        }
        camera.close();
        cv::destroyAllWindows();
    } catch (const std::exception & e) {
        std::cerr << "[main] Exception: " << e.what() << '\n';
        return 1;
    }

    return 0;
}