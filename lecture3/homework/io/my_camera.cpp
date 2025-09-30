#include "my_camera.hpp"
#include <spdlog/spdlog.h>
#include <cstring>

// 构造函数：复刻example.cpp的初始化流程
myCamera::myCamera() {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        spdlog::error("枚举设备失败（无设备或错误）");
        return;
    }
    spdlog::info("发现{}台设备", stDeviceList.nDeviceNum);

    nRet = MV_CC_CreateHandle(&hDev_, stDeviceList.pDeviceInfo[0]);
    if (nRet != MV_OK) {
        spdlog::error("创建句柄失败");
        hDev_ = nullptr;
        return;
    }

    nRet = MV_CC_OpenDevice(hDev_);
    if (nRet != MV_OK) {
        spdlog::error("打开设备失败");
        MV_CC_DestroyHandle(hDev_);
        hDev_ = nullptr;
        return;
    }

    nRet = MV_CC_SetEnumValue(hDev_, "TriggerMode", 0);
    if (nRet != MV_OK) {
        spdlog::warn("设置连续模式失败（非致命）");
    }

    nRet = MV_CC_StartGrabbing(hDev_);
    if (nRet != MV_OK) {
        spdlog::error("开始取流失败");
        MV_CC_CloseDevice(hDev_);
        MV_CC_DestroyHandle(hDev_);
        hDev_ = nullptr;
        return;
    }

    nRet = MV_CC_GetEnumValue(hDev_, "PixelFormat", &stEnumValue_);
    if (nRet == MV_OK) {
        spdlog::info("像素格式获取成功");
    } else {
        spdlog::warn("获取像素格式失败（使用默认）");
    }

    bInit_ = true;
    spdlog::info("相机初始化成功");
}

// 析构函数：复刻example.cpp的资源释放流程
myCamera::~myCamera() {
    if (hDev_ != nullptr) {
        MV_CC_StopGrabbing(hDev_);
        MV_CC_CloseDevice(hDev_);
        MV_CC_DestroyHandle(hDev_);
        hDev_ = nullptr;
    }

    if (pData_ != nullptr) {
        delete[] pData_;
        pData_ = nullptr;
    }

    spdlog::info("相机关闭，资源释放完成");
}

// 读取图像函数：解决MvGvspPixelType类型不匹配问题
bool myCamera::read(cv::Mat& image) {
    if (!bInit_ || hDev_ == nullptr) {
        spdlog::error("相机未初始化，无法取图");
        return false;
    }

    MV_FRAME_OUT stFrameOut;
    memset(&stFrameOut, 0, sizeof(MV_FRAME_OUT));
    int nRet = MV_CC_GetImageBuffer(hDev_, &stFrameOut, 1000);
    if (nRet != MV_OK) {
        spdlog::warn("获取图像缓冲区失败（超时或错误）");
        return false;
    }

    stFrameInfo_ = stFrameOut.stFrameInfo;

    // --- 新版SDK结构体转换（解决类型不匹配） ---
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
    memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));

    stConvertParam.nWidth = stFrameInfo_.nWidth;
    stConvertParam.nHeight = stFrameInfo_.nHeight;
    stConvertParam.pSrcData = stFrameOut.pBufAddr;
    stConvertParam.nSrcDataLen = stFrameInfo_.nFrameLen;
    
    // 关键修改：两次强制转换，先转int再转MvGvspPixelType，解决类型不匹配
    stConvertParam.enSrcPixelType = (MvGvspPixelType)(int)stFrameInfo_.enPixelType;
    stConvertParam.enDstPixelType = (MvGvspPixelType)0x01000003; // MV_PIXEL_FORMAT_BGR888的原始整数值
    
    // 分配目标缓冲区
    unsigned int nDstBufSize = stFrameInfo_.nWidth * stFrameInfo_.nHeight * 3;
    if (pData_ == nullptr) {
        pData_ = new unsigned char[nDstBufSize];
    }
    stConvertParam.pDstBuffer = pData_;
    stConvertParam.nDstBufferSize = nDstBufSize;

    // 调用转换函数
    nRet = MV_CC_ConvertPixelType(hDev_, &stConvertParam);
    // --- 转换结束 ---

    if (nRet != MV_OK) {
        spdlog::error("像素格式转换失败, 错误码: {}", nRet);
        MV_CC_FreeImageBuffer(hDev_, &stFrameOut);
        return false;
    }

    // 转换为OpenCV Mat
    image = cv::Mat(
        stFrameInfo_.nHeight,
        stFrameInfo_.nWidth,
        CV_8UC3,
        pData_
    ).clone();

    // 释放原始缓冲区
    MV_CC_FreeImageBuffer(hDev_, &stFrameOut);

    return !image.empty();
}