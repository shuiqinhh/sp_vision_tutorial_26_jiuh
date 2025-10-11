#include "buff_solver.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace auto_buff
{

Buff_Solver::Buff_Solver()
{
    initCameraParams();
    initModelPoints();
}

void Buff_Solver::initCameraParams()
{
    //相机参数
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
        1286.307063384126 , 0                  , 645.34450819155256, 
                                0                 , 1288.1400736562441 , 483.6163720308021 , 
                                0                 , 0                  , 1    );
    
    //畸变系数
    dist_coeffs_ = (cv::Mat_<double>(5, 1) << -0.47562935060124745, 0.21831745829617311, 0.0004957613589406044, -0.00034617769548693592, 0);
}

void Buff_Solver::initModelPoints()
{
    
    //以扇叶中心为原点建立坐标系，单位：mm


    model_points_3d_.clear();
    
    //扇叶尺寸(注意这里直接用了一半的值)
    float half_width = 160.0f;
    float half_height = 150.0f;
    
    //四个角点
    model_points_3d_.push_back(cv::Point3f(-half_width, -half_height, 0));
    model_points_3d_.push_back(cv::Point3f(half_width, -half_height, 0));
    model_points_3d_.push_back(cv::Point3f(half_width, half_height, 0));
    model_points_3d_.push_back(cv::Point3f(-half_width, half_height, 0));
    
    //扇叶中心
    model_points_3d_.push_back(cv::Point3f(0, 0, 0));
    
    
    model_points_3d_.push_back(cv::Point3f(0, -50, 0));
}

cv::Point3f Buff_Solver::solveFanbladeCenter(const FanBlade& fanblade)
{
    if (fanblade.points.size() < 5) {
        return cv::Point3f(0, 0, 0);
    }
    
    //生成2D图像点
    std::vector<cv::Point2f> image_points;
    for (size_t i = 0; i < fanblade.points.size() && i < model_points_3d_.size(); ++i) {
        image_points.push_back(fanblade.points[i]);
    }
    
    //PnP求解
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        model_points_3d_,
        image_points,
        camera_matrix_,
        dist_coeffs_,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_ITERATIVE
    );
    
    if (!success) {
        return cv::Point3f(0, 0, 0);
    }
    
    //得出扇叶中心在相机坐标系下的3D位置
    return cv::Point3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
}

cv::Point3f Buff_Solver::solveRotationCenter(const std::vector<cv::Point3f>& fanblade_centers)
{
    if (fanblade_centers.empty()) {
        return cv::Point3f(0, 0, 0);
    }
    
    //单个扇叶时的估算
    if (fanblade_centers.size() == 1) {
        cv::Point3f center = fanblade_centers[0];
        float radius = 700.0f; //能量机关中心距扇叶中心700mm
        
        float r_xy = std::sqrt(center.x * center.x + center.y * center.y);
        
        if (r_xy < 1e-6) {
            return cv::Point3f(0, 0, center.z);
        }
        
        float scale = (r_xy - radius) / r_xy;
        return cv::Point3f(center.x * scale, center.y * scale, center.z);
    }
    
    //多个扇叶中心时：最小二乘拟合
    Eigen::MatrixXd A(fanblade_centers.size(), 3);
    Eigen::VectorXd b(fanblade_centers.size());
    
    for (size_t i = 0; i < fanblade_centers.size(); ++i) {
        const auto& pt = fanblade_centers[i];
        A(i, 0) = 2 * pt.x;
        A(i, 1) = 2 * pt.y;
        A(i, 2) = 2 * pt.z;
        b(i) = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
    }
    
    Eigen::Vector3d center = A.colPivHouseholderQr().solve(b);
    
    return cv::Point3f(center(0), center(1), center(2));
}

}  