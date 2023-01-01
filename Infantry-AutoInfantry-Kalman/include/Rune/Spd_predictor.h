/* --------------------------------------------
 * Brief:   使用角度预测角速度的卡尔曼滤波器,
 * Author:  Parker   ——XDU IRobot
 * Date:    2022.12.28
 * -------------------------------------------- */

#ifndef SPD_PREDICTOR_H
#define SPD_PREDICTOR_H

#include <opencv2/opencv.hpp>
#include "Tools.h"
#include "Mat_time.h"
//#include "SpinDetector.h"

typedef struct SpeedTime
{
    double angle_speed; // y
    uint32_t time;      // x

    SpeedTime(double speed = 0.0, uint32_t t = 0)
    {
        angle_speed = speed;
        time = t;
    }
} SpeedTime;

typedef struct AngleTime
{
    double angle;
    uint32_t time;

    AngleTime(double a = 0.0, uint32_t t = 0)
    {
        angle = a;
        time = t;
    }
}AngleTime;

class SpdPredictor
{
private:
    std::shared_ptr<cv::KalmanFilter> KF;//智能指针
    const int StateNum = 2;//状态向量维数
    const int MeasureNum = 1;//观测向量维数
    uint32_t last_t = 0;
    float dt = 0.01;//时间间隔

public:
    /**
     *  @brief  初始化状态值
     *  @param  angle_time   大符速度和线形插值后的时间戳
     */
    void initState(double angle, double speed, uint32_t time);
    
    /**  
     *  @brief  预测
     *  @param  angle_time   大符速度和线形插值后的时间戳
     */
    SpeedTime predict(AngleTime angle_time);

    SpdPredictor();
    ~SpdPredictor() = default;
};

#endif