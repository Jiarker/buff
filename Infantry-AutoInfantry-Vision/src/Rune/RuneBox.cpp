#include "Rune/RuneBox.h"

RuneParam::RuneParam()
{
    FileStorage fs("../Configure/RuneParam.xml", FileStorage::READ);

    fs["approx_epsilon"] >> approx_epsilon;

    fs["blue_brightness_thresh"] >> blue_brightness_thresh;
    fs["blue_color_thresh"] >> blue_color_thresh;

    fs["red_brightness_thresh"] >> red_brightness_thresh;
    fs["red_color_thresh"] >> red_color_thresh;
    fs["blue_red_diff"] >> blue_red_diff;

    //内扇叶
    fs["min_in_vane_area"] >> min_in_vane_area;
    fs["max_in_vane_area"] >> max_in_vane_area;

    fs["min_in_vane_ls_ratio"] >> min_in_vane_ls_ratio;
    fs["max_in_vane_ls_ratio"] >> max_in_vane_ls_ratio;

    fs["min_in_vane_contour_rrect_ratio"] >> min_in_vane_contour_rrect_ratio;
    fs["max_in_vane_contour_rrect_ratio"] >> max_in_vane_contour_rrect_ratio;

    fs["min_in_vane_hull_num"] >> min_in_vane_hull_num;
    fs["max_in_vane_hull_num"] >> max_in_vane_hull_num;

    fs["in_vane_width_offset"] >> in_vane_width_offset;

    //外扇叶
    fs["min_out_vane_area"] >> min_out_vane_area;
    fs["max_out_vane_area"] >> max_out_vane_area;

    fs["min_out_vane_ls_ratio"] >> min_out_vane_ls_ratio;
    fs["max_out_vane_ls_ratio"] >> max_out_vane_ls_ratio;

    fs["min_out_vane_contour_rrect_ratio"] >> min_out_vane_contour_rrect_ratio;
    fs["max_out_vane_contour_rrect_ratio"] >> max_out_vane_contour_rrect_ratio;

    fs["out_vane_width_offset"] >> out_vane_width_offset;

    //内外扇叶
    fs["min_distance_inlongside_ratio"] >> min_distance_inlongside_ratio;
    fs["max_distance_inlongside_ratio"] >> max_distance_inlongside_ratio;

    fs["min_in_out_vane_angle_ratio"] >> min_in_out_vane_angle_ratio;
    fs["max_in_out_vane_angle_ratio"] >> max_in_out_vane_angle_ratio;

    //圆心
    fs["min_R_in_area_ratio"] >> min_R_in_area_ratio;
    fs["max_R_to_predict_distance_ratio"] >> max_R_to_predict_distance_ratio;

    fs["RtoOut_RtoIn"] >> RtoOut_RtoIn;

    fs["max_diff_distance_ratio"] >> max_diff_distance_ratio;

    fs.release();
}

Vane::Vane(vector<Point> vane_contour, float vane_epsilon, float c_area)
{
    contour = vane_contour;
    epsilon = vane_epsilon;
    contour_area = c_area;
    rrect = minAreaRect(contour);
    area = contourArea(contour);
    vector<Point> hull;
    approxPolyDP(contour, hull, epsilon*4, true);//hull*4???
    hull_num = hull.size();
    
    if(rrect.size.height > rrect.size.width)
    {
        short_side = rrect.size.width;
        long_side = rrect.size.height;
    }
    else
    {
        short_side = rrect.size.height;
        long_side = rrect.size.width;
    }
    ls_ratio = long_side / short_side;

    contour_rrect_ratio = contour_area / rrect.size.area();
}

// void Vane::VaneAdjust(float offset, int type)
// {
//     if(type == IN_VANE)//内扇叶修改小边
//     {
//         if(rrect.size.width > rrect.size.height)
//         {
//             if(rrect.size.height - offset > 0)
//                 rrect.size.height -= offset;
//             else
//                 cout<<"offset太大！！！";
//         }
//         else
//         {
//             if(rrect.size.width - offset > 0)
//                 rrect.size.width -= offset;
//             else
//                 cout<<"offset太大！！！";                
//         }
//     }
//     else//外扇叶修改外边
//     {
//         if(rrect.size.width > rrect.size.height)
//         {
//             if(rrect.size.width - offset > 0)
//                 rrect.size.width -= offset;
//             else
//                 cout<<"offset太大！！！";
//         }
//         else
//         {
//             if(rrect.size.height - offset > 0)
//                 rrect.size.height -= offset;
//             else
//                 cout<<"offset太大！！！";                
//         }
//     }
//     //修改内扇叶宽度
    

//     //返回四个顶点
//     rrect.points(points);
// }

RuneArmor::RuneArmor(Vane in, Vane out, GyroPose _gyro_pose, RuneParam param)
{
    in_vane = in;
    out_vane = out;
    gyro_pose = _gyro_pose;
    r_direction = out_vane.rrect.center - in_vane.rrect.center;
    
    //计算圆心坐标
    circle_center =(param.RtoOut_RtoIn / (param.RtoOut_RtoIn - 1)) * in_vane.rrect.center - 
                   (1 / (param.RtoOut_RtoIn - 1)) * out_vane.rrect.center;

    angle = atan2(r_direction.y, r_direction.x);
    //角度转换 
    if(angle < 0)
        angle += 2 * CV_PI;

    // in_vane.VaneAdjust(param.approx_epsilon,IN_VANE);
    // out_vane.VaneAdjust(param.approx_epsilon,OUT_VANE);

    //修改外扇叶宽度
    if(out_vane.rrect.size.height > out_vane.rrect.size.width)
        out_vane.rrect.size.width = in_vane.long_side;
    else
        out_vane.rrect.size.height= in_vane.long_side;
    
    //返回扇叶四个定点
    in_vane.rrect.points(in_vane.points);
    out_vane.rrect.points(out_vane.points);

    //通过angle判断所处象限，进而返回对应角度
    //因后续卡尔曼滤波会改变angle,故在此先计算四点
    if(0 <= angle && angle < CV_PI / 2)
    {
        points[0] = out_vane.points[0];
        points[1] = out_vane.points[3];
        points[2] = in_vane.points[2];
        points[3] = in_vane.points[1];
    }
    else if(CV_PI / 2 <= angle && angle <= CV_PI)
    {
        points[0] = out_vane.points[3];
        points[1] = out_vane.points[2];
        points[2] = in_vane.points[1];
        points[3] = in_vane.points[0];
    }
    else if(CV_PI <= angle && angle <= 3 * CV_PI / 2)
    {
        points[0] = out_vane.points[2];
        points[1] = out_vane.points[1];
        points[2] = in_vane.points[0];
        points[3] = in_vane.points[3];
    }
    else if(3 * CV_PI / 2 <= angle && angle < CV_PI * 2)
    {
        points[0] = out_vane.points[1];
        points[1] = out_vane.points[0];
        points[2] = in_vane.points[3];
        points[3] = in_vane.points[2];
    }

    //计算装甲板圆心
    armor_center = (points[0] + points[1] + points[2] + points[3]) / 4;
}

void RuneArmor::getPoints(vector<cv::Point2f>& pts)
{
    for(int i = 0; i < 4; i++)
        pts.push_back(points[i]);
}