#include "tools.hpp"

cv::Mat resizeAndCenterImage(const cv::Mat& src) {
    const int TARGET_WIDTH = 640;
    const int TARGET_HEIGHT = 640;
 //计算缩放后的图像尺寸
    double scale = std::min(
        static_cast<double>(TARGET_WIDTH) / src.cols,
        static_cast<double>(TARGET_HEIGHT) / src.rows
    );

   
    int newWidth = static_cast<int>(src.cols * scale);
    int newHeight = static_cast<int>(src.rows * scale);

    //计算居中偏移量
    int offsetX = (TARGET_WIDTH - newWidth) / 2;
    int offsetY = (TARGET_HEIGHT - newHeight) / 2;

    //用fmt输出缩放参数
    fmt::print("图像缩放参数:\n");
    fmt::print("原图尺寸: {} × {}\n", src.cols, src.rows);
    fmt::print("缩放因子: {:.2f}\n", scale);
    fmt::print("移量 (X, Y): ({}, {})\n\n", offsetX, offsetY);

    //缩放原图
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(newWidth, newHeight));

    //创建黑色画布
    cv::Mat canvas = cv::Mat::zeros(TARGET_HEIGHT, TARGET_WIDTH, src.type());

    //将缩放后的图像复制到画布中央
    resized.copyTo(canvas(cv::Rect(offsetX, offsetY, newWidth, newHeight)));

    return canvas;
}
    