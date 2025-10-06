#include "my_camera.hpp"
#include <spdlog/spdlog.h>
#include <cstring>

// 构造函数
myCamera::myCamera() {
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (nRet != MV_OK || stDeviceList.nDeviceNum == 0) {
        spdlog::error("查询设备失败");
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
        spdlog::error("开启设备失败");
        MV_CC_DestroyHandle(hDev_);
        hDev_ = nullptr;
        return;
    }

    nRet = MV_CC_SetEnumValue(hDev_, "TriggerMode", 0);
    if (nRet != MV_OK) {
        spdlog::warn("进入连续模式失败");
    }

    nRet = MV_CC_StartGrabbing(hDev_);
    if (nRet != MV_OK) {
        spdlog::error("取流失败");
        MV_CC_CloseDevice(hDev_);
        MV_CC_DestroyHandle(hDev_);
        hDev_ = nullptr;
        return;
    }

    nRet = MV_CC_GetEnumValue(hDev_, "PixelFormat", &stEnumValue_);
    if (nRet == MV_OK) {
        spdlog::info("像素格式转换成功");
    } else {
        spdlog::warn("像素格式转换失败");
    }

    bInit_ = true;
    spdlog::info("初始化成功");
}

// 析构函数
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

    spdlog::info("相机关闭成功");
}

// 读取图像函数：解决MvGvspPixelType类型不匹配问题
bool myCamera::read(cv::Mat& image) {
    if (!bInit_ || hDev_ == nullptr) {
        spdlog::error("相机未初始化");
        return false;
    }

    MV_FRAME_OUT stFrameOut;
    memset(&stFrameOut, 0, sizeof(MV_FRAME_OUT));
    int nRet = MV_CC_GetImageBuffer(hDev_, &stFrameOut, 1000);
    if (nRet != MV_OK) {
        spdlog::warn("图像缓冲获取失败");
        return false;
    }

    stFrameInfo_ = stFrameOut.stFrameInfo;
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
    memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));

    stConvertParam.nWidth = stFrameInfo_.nWidth;
    stConvertParam.nHeight = stFrameInfo_.nHeight;
    stConvertParam.pSrcData = stFrameOut.pBufAddr;
    stConvertParam.nSrcDataLen = stFrameInfo_.nFrameLen;
    stConvertParam.enSrcPixelType = (MvGvspPixelType)(int)stFrameInfo_.enPixelType;
    stConvertParam.enDstPixelType = (MvGvspPixelType)0x01000003; 
    unsigned int nDstBufSize = stFrameInfo_.nWidth * stFrameInfo_.nHeight * 3;
    if (pData_ == nullptr) {
        pData_ = new unsigned char[nDstBufSize];
    }
    stConvertParam.pDstBuffer = pData_;
    stConvertParam.nDstBufferSize = nDstBufSize;
    nRet = MV_CC_ConvertPixelType(hDev_, &stConvertParam);
    if (nRet != MV_OK) {
        spdlog::error("像素格式转换失败", nRet);
        MV_CC_FreeImageBuffer(hDev_, &stFrameOut);
        return false;
    }

    //转为cvmat
    image = cv::Mat(
        stFrameInfo_.nHeight,
        stFrameInfo_.nWidth,
        CV_8UC3,
        pData_
    ).clone();
    MV_CC_FreeImageBuffer(hDev_, &stFrameOut);
    return !image.empty();
}