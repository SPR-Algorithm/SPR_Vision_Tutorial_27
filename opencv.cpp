#include <opencv2/opencv.hpp>

int main() {
    cv::Mat img = cv::imread("armor.png");
    if (img.empty()) {
        std::cerr << "图像读取失败\n";
        return 1;
    }

    std::cout << "尺寸 " << img.cols << "x" << img.rows
              << ",通道数 " << img.channels() << '\n';

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::imshow("gray", gray);
    cv::waitKey(0);
    return 0;
}
 
