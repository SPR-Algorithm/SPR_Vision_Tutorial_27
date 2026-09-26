#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
    cv::Mat image = cv::imread("blue_5.png");
    if (image.empty()) {
        std::cerr << "Error: Could not open image." << std::endl;
        return -1;
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat b_fixed;
    double b = cv::threshold(gray, b_fixed, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
    cv::GaussianBlur(b_fixed, b_fixed, cv::Size(5, 5), 0);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    //cv::Point (-1, -1) means that the anchor point is at the center of the kernel,是规定迭代次数的前提
    cv::Mat open, close;
    cv::morphologyEx(b_fixed, open, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(open, close, cv::MORPH_CLOSE, kernel);
    std::vector<std::vector<cv::Point>> contours;
    //画矩形的全过程
    cv::findContours(close, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        cv::Rect boundingBox = cv::boundingRect(contour);
        cv::rectangle(image, boundingBox, cv::Scalar(0, 255, 0), 2);
    }
    cv::imshow("Detected Objects", image);
    cv::imshow("Binary Image", close);
    cv::waitKey(0);
    return 0;
}