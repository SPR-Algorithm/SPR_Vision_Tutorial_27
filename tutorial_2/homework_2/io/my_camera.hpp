#pragma once
#include<opencv2/opencv.hpp>
#include<string>
#include<memory>
#include "hikrobot/include/MvCameraControl.h"
namespace io{
  class CameraBase{
    public:
    virtual ~CameraBase()=default;
    virtual bool open(const std::string & config)=0;
    virtual bool read(cv::Mat& frame,std::chrono::steady_clock::time_point & timestamp)=0;
    virtual bool close()=0; 
  };
  class HikCamera : public CameraBase{
    public:
      bool open(const std::string & config) override;
      bool read(cv::Mat& frame,std::chrono::steady_clock::time_point & timestamp) override;
      bool close() override;
    private:
      cv::Mat transfer(MV_FRAME_OUT& raw);
      void* handle=nullptr;
  };
  class UsbCamera : public CameraBase{
    public:
      bool open(const std::string & config) override;
      bool read(cv::Mat& frame,std::chrono::steady_clock::time_point & timestamp) override;
      bool close() override;
    private:
      cv::VideoCapture cap;
  };

  class Camera{
    public:
    Camera(const std::string & config);
    
    bool read(cv::Mat & img,std::chrono::steady_clock::time_point & timestamp) 
        { return impl->read(img,timestamp); }
    private:
    std::unique_ptr<CameraBase> impl; 
  };
}