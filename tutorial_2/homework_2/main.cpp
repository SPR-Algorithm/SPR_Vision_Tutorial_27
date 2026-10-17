#include "io/my_camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"
#include "iostream"
#include "chrono"


int main() {
  // 初始化相机、yolo类
  
  io::Camera camera("configs/camera.yaml");
  auto_aim::YOLO yolo("configs/yolo.yaml", true);

  int frame_count = 0;
  int fps_frame_count = 0 ;
  auto fps_last_time = std::chrono::steady_clock::now();
  
  // 调用相机读取图像

  while (true) {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
        camera.read(img, timestamp);
        if (img.empty()) continue; 
        const auto armors = yolo.detect(img, frame_count++);
        
        //fps_count armors_count

        fps_frame_count++;
        auto now  = std :: chrono :: steady_clock ::now();
        double elapsed = std::chrono :: duration<double>(now - fps_last_time).count();
        if (elapsed >=1.0){
          double fps = fps_frame_count / elapsed;
          std :: cout << "fps = "<< fps
                      << " armors = " << armors.size() << "\n";
                      fps_frame_count = 0;
                      fps_last_time = now;
        }

       // 调用yolo识别装甲板
        cv::resize(img,img,cv::Size(640,480));
        cv::imshow("img" , img);
        if (cv::waitKey(1) == 'q') break;
  }
 
 return 0;
}
