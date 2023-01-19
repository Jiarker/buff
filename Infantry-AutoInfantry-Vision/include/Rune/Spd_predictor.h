/* --------------------------------------------
 * Brief:   卡尔曼滤波
 * Author:  Parker   XDU IRobot
 * Date:    2022.12.28
 * -------------------------------------------- */

#ifndef SPD_PREDICTOR_H
#define SPD_PREDICTOR_H

#include <opencv2/opencv.hpp>
#include "Tools.h"
#include "Mat_time.h"


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
    std::shared_ptr<cv::KalmanFilter> KF;
    const int StateNum = 2;
    const int MeasureNum = 1;
    uint32_t last_t = 0;
    float dt = 0.01;

public:
    /**
     *  @brief  卡尔曼滤波初始化
     *  @param  angle   角度
     *  @param  speed   角速度
     *  @param  time    时间戳
     */
    void initState(double angle, double speed, uint32_t time);
    
    /**  
     *  @brief  卡尔曼滤波更新数据
     *  @param  angle   角度
     *  @param  time    时间戳
     */
    double predict(double angle, uint32_t time);

    SpdPredictor();
    ~SpdPredictor() = default;
};

#endif