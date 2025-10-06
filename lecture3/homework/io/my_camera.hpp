#ifndef MY_CAMERA_HPP
#define MY_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"  

class myCamera {
public:
    myCamera();  //构造函数-初始化
    ~myCamera(); //析构函数-释放资源缓存
    bool read(cv::Mat& image);  //读取图像

private:
    void* hDev_ = nullptr;               //句柄
    MV_FRAME_OUT_INFO_EX stFrameInfo_;   //读取帧
    unsigned char* pData_ = nullptr;     //图像数据缓冲区
    bool bInit_ = false;                 
    MVCC_ENUMVALUE stEnumValue_;         //获取像素格式
};

#endif // MY_CAMERA_HPP