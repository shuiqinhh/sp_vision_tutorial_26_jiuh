#include "tasks/buff_detector.hpp"
#include "tasks/buff_solver.hpp"
#include "io/camera.hpp"
#include "tools/plotter.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <deque>

int main()
{
    //相机初始化(对应曝光, 增益, 设备ID)
    io::Camera camera(2.5, 16.9, "2bdf:0001");
    std::chrono::steady_clock::time_point timestamp;  
    
    //导入检测点模型和求解中心模型
    auto_buff::Buff_Detector detector;
    auto_buff::Buff_Solver solver;
    
    //初始化plotjuggler
    tools::Plotter plotter;
    
    //用于平滑旋转中心的历史数据
    std::deque<cv::Point3f> fanblade_center_history;
    const size_t max_history = 10;
    
    while(true){
        cv::Mat img;
        camera.read(img, timestamp);
        
        if (img.empty()) {
            std::cerr << "无法读取相机图像!" << std::endl;
            continue;
        }
        
        //开始检测扇叶
        auto fanblades = detector.detect(img);
        


        //创建显示图像
        cv::Mat display_img = img.clone();
        cv::Point3f fanblade_center_3d(0, 0, 0);
        cv::Point3f rotation_center_3d(0, 0, 0);
        
        // 处理检测到的扇叶
        for (const auto& fanblade : fanblades) {
            cv::Scalar color;
            std::string type_name;
            
            //根据类型设置颜色
            switch (fanblade.type) {
                case auto_buff::_target:
                    color = cv::Scalar(0, 255, 0);  //绿色
                    type_name = "_target";
                    break;
                case auto_buff::_light:
                    color = cv::Scalar(0, 255, 255); //黄色
                    type_name = "_light";
                    break;
                case auto_buff::_unlight:
                    color = cv::Scalar(0, 0, 255);  //红色
                    type_name = "_unlight";
                    break;
            }
            
            //绘制关键点
            for (size_t i = 0; i < fanblade.points.size(); ++i) {
                cv::circle(display_img, fanblade.points[i], 3, color, -1);
                cv::putText(display_img, std::to_string(i), 
                           cv::Point(fanblade.points[i].x + 5, fanblade.points[i].y - 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }
            
            //绘制平面中心点
            cv::circle(display_img, fanblade.center, 5, color, 2);
            cv::putText(display_img, "CENTER", 
                       cv::Point(fanblade.center.x + 10, fanblade.center.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
            
            //解算扇叶中心的3D位置
            fanblade_center_3d = solver.solveFanbladeCenter(fanblade);
            
            //在图像上显示3D位置
            std::string pos_text = cv::format("3D:(%.0f,%.0f,%.0f)mm", 
                fanblade_center_3d.x, fanblade_center_3d.y, fanblade_center_3d.z);
            cv::putText(display_img, pos_text,
                       cv::Point(fanblade.center.x - 80, fanblade.center.y + 25),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
            
            //保存历史数据用于平滑
            fanblade_center_history.push_back(fanblade_center_3d);
            if (fanblade_center_history.size() > max_history) {
                fanblade_center_history.pop_front();
            }
        }
        
        //由扇叶中心推算能量中心
        if (!fanblade_center_history.empty()) {
            std::vector<cv::Point3f> centers(fanblade_center_history.begin(), 
                                            fanblade_center_history.end());
            rotation_center_3d = solver.solveRotationCenter(centers);
            
            //在图像上显示旋转中心
            std::string rot_text = cv::format("R-Center:(%.0f,%.0f,%.0f)mm", 
                rotation_center_3d.x, rotation_center_3d.y, rotation_center_3d.z);
            cv::putText(display_img, rot_text,
                       cv::Point(20, 80),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        }
        
        //显示检测数量到的数量
        std::string count_text = cv::format("Detected: %d", (int)fanblades.size());
        cv::putText(display_img, count_text,
                   cv::Point(20, 120),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        
        //plotjuggler输出
        nlohmann::json data;
        
        //扇叶中心位置曲线
        data["fanblade_center_x"] = fanblade_center_3d.x;
        data["fanblade_center_y"] = fanblade_center_3d.y;
        data["fanblade_center_z"] = fanblade_center_3d.z;
        
        //能量中心位置曲线
        data["rotation_center_x"] = rotation_center_3d.x;
        data["rotation_center_y"] = rotation_center_3d.y;
        data["rotation_center_z"] = rotation_center_3d.z;
        
        //额外信息
        data["detection_count"] = fanblades.size();
        
        plotter.plot(data);
        
        
        //按键控制
        int key = cv::waitKey(1);
        if (key == 'q') {  //q健退出
            break;
        } else if (key == 's' || key == 'S') {  //s键保存图像
            std::string filename = "capture_" + 
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + 
                ".jpg";
            cv::imwrite(filename, display_img);
            std::cout << "图像已保存: " << filename << std::endl;
        }
    }
    
    cv::destroyAllWindows();
    return 0;
}