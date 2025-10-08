#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "io/hikrobot/include/MvCameraControl.h"
#include <fmt/core.h>


int main()
{
    //日志
    tools::logger()->info("Starting main process...");
    //初始化相机
    void* camera_handle = nullptr;
    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
        tools::logger()->error("Failed to find camera devices!");
        return -1;
    }

    //创建相机句柄
    ret = MV_CC_CreateHandle(&camera_handle, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        tools::logger()->error("Failed to create camera handle!");
        return -1;
    }


    //open the camera
    ret = MV_CC_OpenDevice(camera_handle);
    if (ret != MV_OK) {
        tools::logger()->error("Failed to open camera!");
        MV_CC_DestroyHandle(camera_handle);
        return -1;

    }




    //config
    MV_CC_SetEnumValue(camera_handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(camera_handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(camera_handle, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(camera_handle, "ExposureTime", 5000);
    MV_CC_SetFloatValue(camera_handle, "Gain", 20);
    MV_CC_SetFrameRate(camera_handle, 60);






    ret = MV_CC_StartGrabbing(camera_handle);
    if (ret != MV_OK) {
        tools::logger()->error("Failed to start grabbing!");
        MV_CC_CloseDevice(camera_handle);
        MV_CC_DestroyHandle(camera_handle);
        return -1;
    }
    tools::logger()->info("Camera initialized successfully.");

    //YOLO初始化
    auto_aim::YOLO yolo("/home/rm/Desktop/sp_vision_tutorial_26_jiuh/lecture3/homework/configs/yolo.yaml");
    tools::logger()->info("YOLO model initialized successfully.");

    //读取图像+装甲板
    int frame_count = 0;
    while (true) {
        MV_FRAME_OUT raw_frame;
        ret = MV_CC_GetImageBuffer(camera_handle, &raw_frame, 100);
        if (ret != MV_OK) {
            tools::logger()->warn("Failed to get image buffer, retrying...");
            continue;
        }
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





        MV_CC_FreeImageBuffer(camera_handle, &raw_frame);

        //装甲板识别模型
        auto armors = yolo.detect(img, frame_count);
        frame_count++;

        cv::Mat display_img = img.clone();
        for (const auto& armor : armors) {
            // 1. 处理装甲板角点：Point2f（浮点）→ Point（整数），并检查有效性
            std::vector<cv::Point> int_points;  // 用于存储转换后的整数型角点
            bool points_valid = true;

            // 1.1 检查角点集是否为空
            if (armor.points.empty()) {
                tools::logger()->warn("Frame {}: Armor points is empty, skip drawing.", frame_count);
                points_valid = false;
            }
            // 1.2 检查角点数量是否为4（装甲板是四边形，必须4个角点）
            else if (armor.points.size() != 4) {
                tools::logger()->warn("Frame {}: Armor points count is {}, need 4, skip drawing.", 
                                     frame_count, armor.points.size());
                points_valid = false;
            }
            // 1.3 转换浮点角点→整数角点，并检查坐标是否在图像范围内
            else {
                for (const auto& float_p : armor.points) {
                    // 检查坐标是否超出图像边界（避免无效坐标导致绘图错误）
                    if (float_p.x < 0 || float_p.x >= display_img.cols || 
                        float_p.y < 0 || float_p.y >= display_img.rows) {
                        tools::logger()->warn("Frame {}: Invalid point ({:.1f}, {:.1f}) (out of image bounds), skip drawing.", 
                                             frame_count, float_p.x, float_p.y);
                        points_valid = false;
                        int_points.clear();  // 清空无效的点集
                        break;
                    }
                    // 用cvRound()四舍五入转换（比直接截断更精准，避免坐标偏移）
                    int_points.emplace_back(cvRound(float_p.x), cvRound(float_p.y));
                }
            }

            // 2. 绘制装甲板矩形（仅当角点有效时执行）
            if (points_valid) {
                cv::polylines(display_img, int_points, true, cv::Scalar(0, 0, 255), 2); 
            }
            // 角点无效则跳过当前装甲板的后续绘制
            else {
                continue;
            }

            // 3. 绘制装甲板信息（颜色+代号），先检查索引是否合法（避免数组越界崩溃）
            bool info_valid = true;
            std::string armor_color_str, armor_name_str;

            // 3.1 检查color索引是否在COLORS范围内（避免越界访问）
            if (armor.color < 0 || armor.color >= static_cast<int>(auto_aim::COLORS.size())) {
                tools::logger()->warn("Frame {}: Invalid armor color ({}) (out of COLORS range), skip text drawing.", 
                                     frame_count, armor.color);
                info_valid = false;
            }
            else {
                armor_color_str = auto_aim::COLORS[armor.color];
            }

            // 3.2 检查name索引是否在ARMOR_NAMES范围内（避免越界访问）
            if (armor.name < 0 || armor.name >= static_cast<int>(auto_aim::ARMOR_NAMES.size())) {
                tools::logger()->warn("Frame {}: Invalid armor name ({}) (out of ARMOR_NAMES range), skip text drawing.", 
                                     frame_count, armor.name);
                info_valid = false;
            }
            else {
                armor_name_str = auto_aim::ARMOR_NAMES[armor.name];
            }

            // 3.3 仅当信息合法时，绘制文本
            if (info_valid) {
                std::string info = fmt::format("{} {}", armor_color_str, armor_name_str);
                // 注意：armor.center是Point2f，若tools::draw_text要求Point，需同样转换（此处假设支持Point2f，若报错可加cvRound）
                tools::draw_text(display_img, info, armor.center, cv::Scalar(0, 0, 255), 0.8, 2);
            }
        }
        cv::resize(display_img, display_img, cv::Size(640, 480));
        cv::imshow("Armor Detection", display_img);

        //Q键退出程序
        char key = cv::waitKey(1);
        if (key == 'q') {
            tools::logger()->info("Exit command received.");
            break;
        }
    }
    MV_CC_StopGrabbing(camera_handle);
    MV_CC_CloseDevice(camera_handle);
    MV_CC_DestroyHandle(camera_handle);
    cv::destroyAllWindows();
    tools::logger()->info("Main process exited successfully.");

    return 0;
}




