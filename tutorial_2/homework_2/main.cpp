#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include"tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include<sstream>
int main() {
  // 初始化相机、yolo类
   io::Camera C1("configs/camera.yaml");
    io::Camera C("configs/usbcamera.yaml");
    auto_aim::YOLO detector("configs/yolo.yaml");
    cv::Mat img;
while(C.is_open())
{
  // 调用相机读取图像
  C.read(img);
  // 调用yolo识别装甲板
  std::list<auto_aim::Armor>out;
   out=detector.detect(img);
  // 显示图像
  for(auto&aromor:out)
  {
    std::stringstream ss;ss<<"color:"<<[](auto_aim::Color c){
      switch (c)
      {
        case auto_aim::red : return "red";break;
        case auto_aim::blue:return "blue";break;
        case auto_aim::extinguish:return "extinguish";break;
        case auto_aim::purple:return "purple";break;
        default:return "color fail";break;
      }
    }(aromor.color);
    tools::draw_text(img,ss.str(),aromor.center);
    std::vector<cv::Point2f>ps=aromor.points;
    cv::line(img,ps[0],ps[1],cv::Scalar(0,0,255));
  cv::line(img,ps[1],ps[2],cv::Scalar(0,0,255));
   cv::line(img,ps[2],ps[3],cv::Scalar(0,0,255)); 
   cv::line(img,ps[3],ps[0],cv::Scalar(0,0,255)); 
  }
   cv::resize(img, img , cv::Size(640, 480));
   cv::imshow("img", img);
   if (cv::waitKey(37) == 27)break;
  }
 return 0;
}