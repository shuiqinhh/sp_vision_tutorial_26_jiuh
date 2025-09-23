#include <opencv2/opencv.hpp>
#include "tools.hpp"
#include <fmt/core.h>  

int main() { 
    //图像路径
    std::string img_path = "/home/rm/Desktop/sp_hw/sp_vision_tutorial_26_jiuh/lecture2/img/test_1.jpg";
    
    cv::Mat image = cv::imread(img_path);
    cv::Mat result = resizeAndCenterImage(image);
    cv::imshow("处理图像", result);
    cv::waitKey(0); 
    cv::destroyAllWindows();
    return 0;
}