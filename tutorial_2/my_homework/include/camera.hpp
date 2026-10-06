#include<string>
#include<vector>
#include<iostream>
#include<memory>
#include<opencv2/opencv.hpp>
struct Frame
{
    explicit Frame(int r_=0,int c_=0):rows(r_),cols(c_),color(rows*cols*3,0){}
    size_t rows;
    size_t cols;
    std::vector<uint8_t>color;
};
class Camera
{
    public:
    virtual ~Camera()=default;
    virtual void open()=0;
    virtual bool read(Frame&)=0;
    virtual void close()=0;
    virtual std::string name ()const=0;
};
class UsbCamera:public Camera
{
    public:
    explicit UsbCamera(std::string id_):id(id_),status(false),frame_(0){}
    ~UsbCamera()override{close();};
    void open()override;
    bool read(Frame&)override;
    void close()override;
    std::string name() const override;
    private:
    std::string id;
    bool status;
    size_t frame_;
};
class IndustrialCamera:public Camera
{
      public:
      explicit IndustrialCamera(std::string id_):id(id_),status(false),frame_(0){}
      ~IndustrialCamera()override{close();}
      void open()override;
      bool read(Frame&)override;
      void close()override;
      std::string name()const override;
      private:
      std::string id;
      bool status;
      size_t frame_;
};
extern std::unique_ptr<Camera>makeCamera(std::string,std::string);
