/*-------------------------------------------------------------
brief  :  RuneDetector
author :  XDU Robomaster, Jiarker
date   :  2023.1.13
---------------------------------------------------------------*/

#ifndef RUNEBOX_H
#define RUNEBOX_H

#include <chrono>
#include "Tools.h"
#include "Mat_time.h"
#include <iostream>

using namespace std;
using namespace cv;

#define IN_VANE 1
#define OUT_VANE 2

typedef struct RuneParam
{
    float approx_epsilon;                       // 多边形拟合参数

    float blue_brightness_thresh;
    float blue_color_thresh;

    float red_brightness_thresh;
    float red_color_thresh;
    float blue_red_diff;
    
    //内扇叶
    float min_in_vane_area;                     //  面积
    float max_in_vane_area;

    float min_in_vane_ls_ratio;                 //  长短边比
    float max_in_vane_ls_ratio;

    float min_in_vane_contour_rrect_ratio;      //  扇叶轮廓面积 / 外接矩形面积
    float max_in_vane_contour_rrect_ratio;

    float min_in_vane_hull_num;                 //  轮廓角点数
    float max_in_vane_hull_num;

    //外扇叶
    float min_out_vane_area;                     //  面积
    float max_out_vane_area;

    float min_out_vane_ls_ratio;                 //  长短边比
    float max_out_vane_ls_ratio;

    float min_out_vane_contour_rrect_ratio;      //  扇叶轮廓面积 / 外接矩形面积
    float max_out_vane_contour_rrect_ratio;

    //内外扇叶
    float min_distance_inlongside_ratio;         //  内外扇叶中心点距离 / 内扇叶长边
    float max_distance_inlongside_ratio; 

    float max_in_out_vane_derta_angle;           //  旋转角度差，注意在x或y轴时角度突变
                                                 //  两不同扇叶间夹角为72度
                                                 //  旋转外接矩形夹角是角度制，圆心到外扇叶与x轴夹角为弧度制

    //圆心
    //暂时不对最大距离进行判断
    float min_R_in_area_ratio;                   // R标志面积与内扇叶外接矩形面积比
    float max_R_to_predict_distance_ratio;       // 预估圆心(计算得到)与R(测量得到)的距离 / 外扇叶长边

    float RtoOut_RtoIn;                          // R到外扇叶中心距离 / R到内扇叶中心距离 

    float max_diff_distance_ratio;               // 新旧外扇叶中心距离 / 外扇叶长边

    RuneParam();
}RuneParam;

class Vane
{
public:   
    RotatedRect rrect;      //  最小外接矩形
    float short_side;       //  短边
    float long_side;        //  长边

    Point2f points[4];      //  rrect的四个顶点

public:
    ~Vane() = default;
    Vane() = default;
    Vane(RotatedRect c_rrect, float s_side, float l_side)
    {
        rrect = c_rrect;
        short_side = s_side;
        long_side = l_side;
    };
};


class RuneArmor
{
public:
    Vane in_vane;                               // 内扇叶
    Vane out_vane;                              // 外扇叶

    Point2f r_direction;                        // 半径方向，圆心指向装甲板

    float angle;                               // 角度，单位：弧度    
    GyroPose gyro_pose;                         // 时间戳

    cv::Point2f circle_center;                  // 能量机关圆心 
    vector<Point2f> points;                          // 返回的四个顶点
    Point2f armor_center;                       // 装甲板圆心

public:
    ~RuneArmor() = default;
    RuneArmor() = default;
    RuneArmor(Vane in_vane, Vane out_vane, GyroPose _gyro_pose, RuneParam param);

    /**
    *  @brief   获取四个角点，顺时针顺序
    * @param    pts 接收装甲板四顶点
    */ 
    void getPoints(vector<cv::Point2f>& pts);

private:
    /**
     * @brief   将距离vane1中心最短的vane2外接矩形两顶点返回给points
     * @param   vane1   提供中心
     * @param   vane2   提供顶点 
    */
   void getPoints_first(Vane vane1, Vane vane2);

   /**
    * @brief    交换定点位置
    * @param    point1  
    * @param    point2
    * @param    type    扇叶类型
   */
   void exchange(Point2f &point1, Point2f &point2, int type);
};


#endif