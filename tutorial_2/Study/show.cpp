#include<opencv2/opencv.hpp>
#include<vector>
#include<iostream>
int main(){
    //放图
    /*cv::Mat img = cv::imread("blue_5.png");
    cv::imshow("img",img);
    cv::waitKey(0);*/

    //分离3通道
    /*cv::Mat yuantu=cv::imread("blue_5.png");
    std::vector<cv::Mat> tongdao;
    cv::split(yuantu,tongdao);
    cv::imshow("B",tongdao[0]);
    cv::imshow("G",tongdao[1]);
    cv::imshow("R",tongdao[2]);
    cv::waitKey(0);*/

    //绘制矩形
    /*cv::Mat yuantu=cv::imread("blue_5.png");
    cv::Rect juxing;
    juxing.x=0;
    juxing.y=0;
    juxing.width=100;
    juxing.height=100;
    cv::Mat tu=yuantu.clone();
    rectangle(tu,juxing,cv::Scalar(0,0,255),4,8,0);
    cv::imshow("Rect",tu);
    cv::waitKey(0);*/

    //画直线
    /*cv::Mat yuantu=cv::imread("blue_5.png");
    cv::Mat tu=yuantu.clone();
    line(tu,cv::Point(0,0),cv::Point(100,100),cv::Scalar(0,0,255),1,cv::LINE_AA,0);
    cv::imshow("line",tu);
    cv::waitKey(0);*/

    //视频
    /*cv::VideoCapture cap(0,cv::CAP_V4L2);
    //cv::VideoCapture cap(0);
    std::cout<<"0"<<std::endl;
    if (!cap.isOpened()) {
        std::cerr << "错误：无法打开摄像头 /dev/video0" << std::endl;
        return -1;
    }
    std::cout<<"0"<<std::endl;
    cv::Mat frame;
    std::cout<<"0"<<std::endl;
    //cv::VideoCapture cap("/home/rsve/test/SPR_Vision_Tutorial_27/tutorial_2/Study/demo.mp4");
    while(true){
        
        cap >> frame;

        if (frame.empty()) {
        std::cerr << "Warning: Empty frame captured. End of video or read error." << std::endl;
        break; // 退出循环，避免崩溃
        }

        cv::imshow("sxt",frame);
            cv::waitKey(1)==27;
    }
    */
   
    //阈值
    /*cv::Mat yuantu=cv::imread("blue_5.png");
    cv::Mat tu=yuantu.clone();
    cv::Mat gray;
    cv::cvtColor(tu,gray,cv::COLOR_BGR2GRAY);//灰度图
    cv::Mat GaussianBlur;
    cv::GaussianBlur(gray,GaussianBlur, {5, 5}, 0);//高斯降噪
    cv::Mat edge;
    cv::Canny(GaussianBlur,edge,50,150);
    cv::imshow("gray",gray);
    cv::imshow("GaussianBlur",GaussianBlur);
    cv::imshow("edge",edge);
    cv::waitKey(0);
    return 0; */

    //灰度图+高斯降噪+二值化
    cv::Mat frame;
    cv::VideoCapture cap(0,cv::CAP_V4L2);
    while(true){
    cap>>frame;
    cv::Mat gray;
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);//灰度图
    cv::Mat GaussianBlur;
    cv::GaussianBlur(gray,GaussianBlur, {5, 5}, 0);//高斯降噪
    cv::Mat edge;
    cv::Canny(GaussianBlur,edge,50,150);//edge
    cv::imshow("gray",gray);
    cv::imshow("GaussianBlur",GaussianBlur);
    cv::imshow("edge",edge);
    cv::Mat binary;
    cv::threshold(GaussianBlur,binary,120,255,cv::THRESH_BINARY);//二值化
    imshow("binary",binary);
        cv::waitKey(30);
    }

    /*std::vector<std::vector<cv::Point>>contours;
    cv::findContours(binary,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    std::vector<cv::RotateRect>rotated;
    for(const auto&contour:contours){
        auto rotated=cv::minAreaRect(contour);
        rotated.emplace_back(rotated_rect);
    }*/
    return 0; 
}  