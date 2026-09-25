#include<opencv2/opencv.hpp>
int main(){
    auto img = cv::imread("blue_5.png");
    cv::imshow("img",img);
    cv::waitKey(0);
    return 0; 
}