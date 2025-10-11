#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "buff_type.hpp"

namespace auto_buff
{
class Buff_Solver
{
public:
    Buff_Solver();
    
    //解算扇叶中心的位置
    cv::Point3f solveFanbladeCenter(const FanBlade& fanblade);
    
    //推算算能灵中心
    cv::Point3f solveRotationCenter(const std::vector<cv::Point3f>& fanblade_centers);

private:
    //相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    //扇叶的模型点
    std::vector<cv::Point3f> model_points_3d_;
    
    //初始化
    void initCameraParams();
    void initModelPoints();
};
} 
#endif  