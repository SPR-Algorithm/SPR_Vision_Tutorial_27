#include <opencv2/opencv.hpp>
int main(){
    cv::Mat img = cv::imread("blue_5.png");
    //读取BGR三通道的图片
    if (img.empty()){
        std::cerr <<"错误"<<"\n";
        return -1;
    }
    cv::imwrite("output.jpg", img);//写入
    cv::imshow("窗口名字",img);//展示
    cv::Mat hsv,blue;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);//转换为HSV颜色空间
    cv::inRange(hsv, cv::Scalar(100, 100, 100), cv::Scalar(120, 255, 255), hsv);//提取蓝色区域
    cv::bitwise_and(img, img, blue, hsv);//将提取的蓝色区域与原图进行按位与操作
    cv::imshow("提取的蓝色区域", blue);
    cv::waitKey(0);
    return 0;
}