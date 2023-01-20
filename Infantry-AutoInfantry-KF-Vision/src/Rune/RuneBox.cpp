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

    //外扇叶
    fs["min_out_vane_area"] >> min_out_vane_area;
    fs["max_out_vane_area"] >> max_out_vane_area;

    fs["min_out_vane_ls_ratio"] >> min_out_vane_ls_ratio;
    fs["max_out_vane_ls_ratio"] >> max_out_vane_ls_ratio;

    fs["min_out_vane_contour_rrect_ratio"] >> min_out_vane_contour_rrect_ratio;
    fs["max_out_vane_contour_rrect_ratio"] >> max_out_vane_contour_rrect_ratio;

    //内外扇叶
    fs["min_distance_inlongside_ratio"] >> min_distance_inlongside_ratio;
    fs["max_distance_inlongside_ratio"] >> max_distance_inlongside_ratio;

    fs["max_in_out_vane_derta_angle"] >> max_in_out_vane_derta_angle;

    //圆心
    fs["min_R_in_area_ratio"] >> min_R_in_area_ratio;
    fs["max_R_to_predict_distance_ratio"] >> max_R_to_predict_distance_ratio;

    fs["RtoOut_RtoIn"] >> RtoOut_RtoIn;

    fs["max_diff_distance_ratio"] >> max_diff_distance_ratio;

    fs.release();
}


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
    //cout<<"angle:"<<angle<<endl;
}

void RuneArmor::getPoints(vector<cv::Point2f>& pts)
{
    // //修改内扇叶宽度
    // if(in_vane.rrect.size.height > in_vane.rrect.size.width)
    //     in_vane.rrect.size.width = out_vane.long_side;
    // else
    //     in_vane.rrect.size.height= out_vane.long_side;
    
    //返回扇叶四个顶点
    in_vane.rrect.points(in_vane.points);
    out_vane.rrect.points(out_vane.points);

    //初步得到装甲板四个顶点
    getPoints_first(in_vane, out_vane);
    getPoints_first(out_vane, in_vane);

    //计算装甲板圆心
    armor_center = (points[0] + points[1] + points[2] + points[3]) / 4;

    exchange(points[0], points[1], OUT_VANE);
    exchange(points[2], points[3], IN_VANE);

    for(int i = 0; i < 4; i++)
        pts.push_back(points[i]);
}

void RuneArmor::getPoints_first(Vane vane1, Vane vane2)
{
    //找外扇叶两点，即距离内扇叶中心最近的外扇叶外接矩形两顶点
    int min_1_i = 0, min_2_i = 0;
    float min_1_distance = 999999, min_2_distance = 999999;
    for(int i = 0; i < 4; i++)
    {
        float distance = calDistance(vane1.rrect.center, vane2.points[i]);
        //cout<<"i:"<<i<<"\tdistance:"<<distance<<endl;
        if(distance < min_1_distance)
        {
            //注意覆盖问题
            min_2_i = min_1_i;
            min_2_distance = min_1_distance;
            min_1_distance = distance;
            min_1_i = i;
        }
        else if(distance < min_2_distance)
        {
            min_2_distance = distance;
            min_2_i = i;
        }
    }
    // cout<<"min_1_i:"<<min_1_i<<"\tdistance:"<<min_1_distance<<endl;
    // cout<<"min_2_i:"<<min_2_i<<"\tdistance:"<<min_2_distance<<endl;
    // cout<<endl;
    points.push_back(vane2.points[min_1_i]);
    points.push_back(vane2.points[min_2_i]);
}

void RuneArmor::exchange(Point2f &point1, Point2f &point2, int type)
{
    //得到以装甲板圆心为起点，装甲板四顶点为终点的矢量
    Point2f temp_point;
    temp_point = point1 - armor_center; 
    float temp_angle = atan2(temp_point.y, temp_point.x);
    //得到从x轴旋转至矢量的角度
    if(temp_angle < 0)
        temp_angle += 2 * CV_PI;
    //得到r_direction旋转至矢量的角度，为正顺时针，为负逆时针
    temp_angle -= angle;
    if(temp_angle > CV_PI)
        temp_angle -= 2 * CV_PI;
    else if(temp_angle < -CV_PI)
        temp_angle += 2 * CV_PI;

    if(temp_angle > 0 && type == OUT_VANE)
    {
        Point2f temp_point = point1;
        point1 = point2;
        point2 = temp_point;
    }    

    else if(temp_angle < 0 && type == IN_VANE)
    {
        Point2f temp_point = point1;
        point1 = point2;
        point2 = temp_point;
    }   
}