#ifndef MY_CAMERA_HPP
#define MY_CAMERA_HPP

#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"  // 海康SDK头文件

class myCamera {
public:
    myCamera();  // 构造函数：初始化相机
    ~myCamera(); // 析构函数：释放资源
    bool read(cv::Mat& image);  // 读取一帧图像

private:
    // 私有成员变量名全部以 '_' 结尾，符合作业要求
    void* hDev_ = nullptr;               // 相机句柄 (原: hDev)
    MV_FRAME_OUT_INFO_EX stFrameInfo_;   // 帧信息 (原: stFrameInfo)
    unsigned char* pData_ = nullptr;     // 图像数据缓冲区 (原: pData)
    bool bInit_ = false;                 // 初始化状态标志 (原: bInit)
    MVCC_ENUMVALUE stEnumValue_;         // 枚举值，用于获取像素格式 (原: stEnumValue)
};

#endif // MY_CAMERA_HPP