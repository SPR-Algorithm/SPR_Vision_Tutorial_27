#include"my_camera.hpp"
io::HikCamera::HikCamera(double exposure_ms,double gain_,std::string vid_pid_):exposure(exposure_ms),gain(gain_)
{
int index=vid_pid_.find(":");
try
{
    std::string check=vid_pid_.substr(index+1);int check_index=check.find(":");
    if(check_index!=-1)tools::logger()->warn("invalid vid_pid: \"{}\"",vid_pid_);
    vid=std::stoi(vid_pid_.substr(0,index),nullptr,16);
    pid=std::stoi(vid_pid_.substr(index+1),nullptr,16);
}catch(const std::exception&)
{
  tools::logger()->warn("invalid vid-pid: \"{}\"",vid_pid_);
}
open();
};
void io::HikCamera::open()
{
  MV_CC_DEVICE_INFO_LIST list;
  auto ret=MV_CC_EnumDevices(MV_USB_DEVICE,&list);
  if(ret!=MV_OK)tools::logger()->warn("Dont't find the devicelist");
  auto*device_=pick(list,vid,pid);
  if(device_==nullptr){tools::logger()->warn("don't find the device");return;}
  ret=MV_CC_CreateHandle(&handle,device_);
  if(ret!=MV_OK){tools::logger()->warn("connect fail");return;}
  ret=MV_CC_OpenDevice(handle);
  if(ret!=MV_OK){tools::logger()->warn("open fail");return;}
  MV_CC_SetEnumValue(handle,"BalanceWhiteAuto",MV_BALANCEWHITE_AUTO_CONTINUOUS);
  MV_CC_SetEnumValue(handle,"ExposureAuto",MV_EXPOSURE_AUTO_MODE_OFF);
  MV_CC_SetEnumValue(handle,"GainAuto", MV_GAIN_MODE_OFF);
  MV_CC_SetFloatValue(handle,"ExposureTime",exposure);
  MV_CC_SetFloatValue(handle,"Gain",gain);
  MV_CC_SetFrameRate(handle,60);
};
bool io::HikCamera::is_open()
{
    if(handle=nullptr)return false;
    else return false;
}
void io::HikCamera::read(cv::Mat&M)
{
    if(M.channels()!=1)cv::cvtColor(M,M,cv::COLOR_RGB2GRAY);
    if(!is_open())tools::logger()->warn("camera don't be opened");
    auto ret=MV_CC_StartGrabbing(handle);
    if(ret!=MV_OK)tools::logger()->warn("picture can't be captured");
    MV_FRAME_OUT raw;
    unsigned int  nMsec=100;
    ret =MV_CC_GetImageBuffer(handle,&raw,nMsec);
    if(ret!=MV_OK)tools::logger()->error("read fail");
    cv::resize(M,M,cv::Size(raw.stFrameInfo.nWidth,raw.stFrameInfo.nHeight));
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cvt_param.nWidth=raw.stFrameInfo.nWidth;
    cvt_param.nHeight=raw.stFrameInfo.nHeight;
    cvt_param.pSrcData=raw.pBufAddr;
    cvt_param.nSrcDataLen=raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType=raw.stFrameInfo.enPixelType;
    cvt_param.pDstBuffer=M.data;
    cvt_param.nDstBufferSize = M.total() * M.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    auto pixel_type=raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType,cv::ColorConversionCodes>type_map=
    {
        {{PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
      {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
      {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
      {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}
    }
};
    cv::cvtColor(M,M,type_map.at(pixel_type));
}
MV_CC_DEVICE_INFO* io::HikCamera::pick(MV_CC_DEVICE_INFO_LIST&list,int vid,int pid)
{
    if(list.nDeviceNum==0){tools::logger()->warn("Don't find the device");return nullptr;}
    if(vid==-1&&pid==-1){tools::logger()->warn("you don't choose the device"); return list.pDeviceInfo[0];}
    else
    {
        for(int i=0;i<list.nDeviceNum;++i)
        {
            auto*info=list.pDeviceInfo[i];
            if(info->SpecialInfo.stUsb3VInfo.idVendor!=vid||list.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.idProduct!=pid)continue;
            else return info;
        }
        return nullptr;
    }
}
void io::HikCamera::close()
{
    auto r1=MV_CC_StopGrabbing(handle);
    auto r2=MV_CC_CloseDevice(handle);
    auto r3=MV_CC_DestroyHandle(handle);
    if(r1!=MV_OK||r2!=MV_OK||r3!=MV_OK)tools::logger()->warn("close fail");
    handle=nullptr;
}
io::UsbCamera::UsbCamera(double exposure_,double gain_,std::string vid_pid):
exposure(exposure_),gain(gain_)
{
int index=vid_pid.find(":");
try
{
std::string index_check=vid_pid.substr(index+1);
int index_=index_check.find(":");
if(index_!=-1)tools::logger()->warn("invalidvid_pid \"{}\"",vid_pid);
vid=std::stoi(vid_pid.substr(0,index),nullptr,16);
pid=std::stoi(vid_pid.substr(index+1),nullptr,16);
}
catch(std::exception&)
{tools::logger()->warn("invalid vid_pid: \"{}\"",vid_pid);}
device_id=pick_device();
open();
}
io::UsbCamera::UsbCamera(double exposure_,double gain_,unsigned int device):
exposure(exposure_),gain(gain_),device_id(device){open();}
unsigned int io::UsbCamera::pick_device(){}
void io::UsbCamera::open()
{
    if(device_id==0)tools::logger()->warn("you dont't chose other device");
    cat.open(device_id);
    if(!cat.isOpened())tools::logger()->warn("camera isn't be open,check your device_id or vid_pid again");
    cat.set(cv::CAP_PROP_AUTO_EXPOSURE,0.25);
    cat.set(cv::CAP_PROP_EXPOSURE,exposure);
    cat.set(cv::CAP_PROP_GAIN,gain);
}
bool io::UsbCamera::is_open(){return cat.isOpened();}
void io::UsbCamera::read(cv::Mat&M){cat.read(M);}
io::Camera::Camera(std::string configs_path)
{
    const YAML::Node yaml=tools::load(configs_path);
    auto name=tools::read<std::string>(yaml,"camera_name");
    auto exposure=tools::read<double>(yaml,"exposure_ms");
    auto gain=tools::read<double>(yaml,"gain");
    std::string id=tools::read<std::string>(yaml,"vid_pid");
    if(name=="hikrobot")
    {
        Cam=std::make_unique<HikCamera>(exposure,gain,id);
    }
    if(name=="UsbCamera")
    {
        int device_id=tools::read<int>(yaml,"device_id");
        if(device_id>=0)Cam=std::make_unique<UsbCamera>(exposure,gain,device_id);
        else if(id!="NO")Cam=std::make_unique<UsbCamera>(exposure,gain,id);
        else 
        {
            tools::logger()->warn("don't find the device");
            Cam=std::make_unique<UsbCamera>(exposure,gain,0);
        }
    }
}
bool io::Camera::is_open()
{
    return Cam->is_open();
}
void io::Camera::read(cv::Mat&M)
{
   Cam->read(M);
}