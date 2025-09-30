#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "io/hikrobot/include/MvCameraControl.h"
#include <fmt/core.h>

int main()
{
    // 初始化日志工具
    tools::logger()->info("Starting main process...");

    // 1. 初始化相机
    void* camera_handle = nullptr;
    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        tools::logger()->error("Failed to find camera devices!");
        return -1;
    }

    // 创建相机句柄
    ret = MV_CC_CreateHandle(&camera_handle, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        tools::logger()->error("Failed to create camera handle!");
        return -1;
    }

    // 打开相机
    ret = MV_CC_OpenDevice(camera_handle);
    if (ret != MV_OK) {
        tools::logger()->error("Failed to open camera!");
        MV_CC_DestroyHandle(camera_handle);
        return -1;
    }

    // 配置相机参数
    MV_CC_SetEnumValue(camera_handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(camera_handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(camera_handle, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(camera_handle, "ExposureTime", 10000);
    MV_CC_SetFloatValue(camera_handle, "Gain", 20);
    MV_CC_SetFrameRate(camera_handle, 60);

    // 开始取流
    ret = MV_CC_StartGrabbing(camera_handle);
    if (ret != MV_OK) {
        tools::logger()->error("Failed to start grabbing!");
        MV_CC_CloseDevice(camera_handle);
        MV_CC_DestroyHandle(camera_handle);
        return -1;
    }
    tools::logger()->info("Camera initialized successfully.");

    // 2. 初始化YOLO模型
    auto_aim::YOLO yolo("config/yolo_config.yaml");
    tools::logger()->info("YOLO model initialized successfully.");

    // 3. 循环读取图像并识别装甲板
    int frame_count = 0;
    while (true) {
        // 读取一帧图像
        MV_FRAME_OUT raw_frame;
        ret = MV_CC_GetImageBuffer(camera_handle, &raw_frame, 100);
        if (ret != MV_OK) {
            tools::logger()->warn("Failed to get image buffer, retrying...");
            continue;
        }

        // 转换为OpenCV格式
        cv::Mat img;
        try {
            MV_CC_PIXEL_CONVERT_PARAM cvt_param;
            cvt_param.nWidth = raw_frame.stFrameInfo.nWidth;
            cvt_param.nHeight = raw_frame.stFrameInfo.nHeight;
            cvt_param.pSrcData = raw_frame.pBufAddr;
            cvt_param.nSrcDataLen = raw_frame.stFrameInfo.nFrameLen;
            cvt_param.enSrcPixelType = raw_frame.stFrameInfo.enPixelType;
            cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

            img.create(cv::Size(raw_frame.stFrameInfo.nWidth, raw_frame.stFrameInfo.nHeight), CV_8UC3);
            cvt_param.pDstBuffer = img.data;
            cvt_param.nDstBufferSize = img.total() * img.elemSize();

            ret = MV_CC_ConvertPixelType(camera_handle, &cvt_param);
            if (ret != MV_OK) {
                tools::logger()->warn("Pixel conversion failed, using fallback.");
                img = cv::Mat(cv::Size(raw_frame.stFrameInfo.nWidth, raw_frame.stFrameInfo.nHeight), 
                             CV_8U, raw_frame.pBufAddr);
                const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
                    {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
                    {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
                    {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
                    {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}
                };
                cv::cvtColor(img, img, type_map.at(raw_frame.stFrameInfo.enPixelType));
            }
        } catch (const std::exception& e) {
            tools::logger()->error("Image conversion error: {}", e.what());
            MV_CC_FreeImageBuffer(camera_handle, &raw_frame);
            continue;
        }

        // 释放原始图像缓冲区
        MV_CC_FreeImageBuffer(camera_handle, &raw_frame);

        // 4. YOLO识别装甲板
        auto armors = yolo.detect(img, frame_count);
        frame_count++;

        // 5. 绘制识别结果（颜色+数字/名称，红色线框）
        cv::Mat display_img = img.clone();
        for (const auto& armor : armors) {
            // 绘制红色闭合矩形（连接关键点）
            cv::polylines(display_img, armor.points, true, cv::Scalar(0, 0, 255), 2);

            // 绘制装甲板信息：颜色 + 名称（如 "blue four"）
            std::string info = fmt::format("{} {}", 
                auto_aim::COLORS[armor.color],  // 颜色（如 "blue"）
                auto_aim::ARMOR_NAMES[armor.name]);  // 名称（如 "four"）
            
            // 在装甲板中心绘制文本（红色）
            tools::draw_text(display_img, info, armor.center, cv::Scalar(0, 0, 255), 0.8, 2);
        }

        // 显示图像
        cv::resize(display_img, display_img, cv::Size(640, 480));
        cv::imshow("Armor Detection", display_img);

        // 按'q'退出
        char key = cv::waitKey(1);
        if (key == 'q') {
            tools::logger()->info("Exit command received.");
            break;
        }
    }

    // 6. 资源释放
    MV_CC_StopGrabbing(camera_handle);
    MV_CC_CloseDevice(camera_handle);
    MV_CC_DestroyHandle(camera_handle);
    cv::destroyAllWindows();
    tools::logger()->info("Main process exited successfully.");

    return 0;
}