#include "Solver.h"

/**
  * @brief 进行PNP解算
  * @param _points_2D 进行PNP解算的四个角点
  * @param _box_size  装甲板的类型，不明确时，默认值为小装甲板
 */
bool Solver::solve(const std::vector<cv::Point2f> &_points_2D ,const BoxSize &_box_size){
    set2DPoints(_points_2D);
    set3DPoints(_box_size);
    cv::solvePnP(points_3D,points_2D,param.camera_matrix,param.dist_coeffs,rVec,tVec,false,cv::SOLVEPNP_ITERATIVE);
    double x = tVec.at<double>(0, 0), y = tVec.at<double>(1, 0), z = tVec.at<double>(2, 0);
    point_3D = cv::Point3f((float)x, (float)y, (float)z);
    yaw = atan(x / z) * 180.0 / CV_PI;
    pitch = atan(-y / z) * 180.0 / CV_PI;
    distance = sqrt(x*x + y*y + z*z)/1000.0;
    return true; 
}

/**
  * @brief 设置3D点集
  * @param _box_size  装甲板的类型
 */
void Solver::set3DPoints(const BoxSize &_box_size){
    if(box_size==_box_size&&!points_3D.empty())
        return ;
    
    else
    {
        points_3D.clear();
        if(box_size != RUNE_ARMOR)
        {    
            int width,height;
            box_size=_box_size;
            if(box_size==BIG)
            {
                width=param.big_armor_boxes_real_width;
                height=param.big_armor_boxes_real_height;
            }
            else if(box_size == SMALL)
            {
                width=param.small_armor_boxes_real_width;
                height=param.small_armor_boxes_real_height;
            }
            // std::cout << "Height: " << height << std::endl;
            points_3D.push_back(cv::Point3f(-width/2.0,-height/2.0,0));
            points_3D.push_back(cv::Point3f(width/2.0,-height/2.0,0));
            points_3D.push_back(cv::Point3f(width/2.0,height/2.0,0));
            points_3D.push_back(cv::Point3f(-width/2.0,height/2.0,0));
        }
        else
        {
            int width_in, width_out, height;
            width_in = param.rune_armor_boxes_real_width_in;
            width_out = param.rune_armor_boxes_real_width_out;
            height = param.rune_armor_boxes_real_height;
            points_3D.push_back(cv::Point3f(-width_out/2.0,-height/2.0,0));
            points_3D.push_back(cv::Point3f(width_out/2.0,-height/2.0,0));
            points_3D.push_back(cv::Point3f(width_in/2.0,height/2.0,0));
            points_3D.push_back(cv::Point3f(-width_in/2.0,height/2.0,0));
        }
    }
}

/**
 * @brief 设置2D点集
 * @param _points_2D 进行PNP解算的四个角点
*/
void Solver::set2DPoints(const std::vector<cv::Point2f> &_points_2D){
    points_2D.clear();
    points_2D=_points_2D;
}


SolverParam::SolverParam()
{
    
    cv::FileStorage fs("../Configure/Settings.xml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs["small_armor_boxes_real_height"] >> small_armor_boxes_real_height;
    fs["small_armor_boxes_real_width"] >> small_armor_boxes_real_width;
    fs["big_armor_boxes_real_height"] >> big_armor_boxes_real_height;
    fs["big_armor_boxes_real_width"] >> big_armor_boxes_real_width;
    fs["rune_armor_boxes_real_height"] >> rune_armor_boxes_real_height;
    fs["rune_armor_boxes_real_width_in"] >> rune_armor_boxes_real_width_in;
    fs["rune_armor_boxes_real_width_out"] >> rune_armor_boxes_real_width_out;

    fs.release();
}