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
#include "EKF.h"

typedef struct AngleSpeedTime
{
    double angle;
    double speed;
    uint32_t time;

    AngleSpeedTime(double a = 0.0, double s = 0.0, uint32_t t = 0)
    {
        angle = a;
        speed = s;
        time = t;
    }
}AngleSpeedTime;

class SpdPredictor
{
private:
    std::shared_ptr<cv::KalmanFilter> KF;//智能指针
    const int StateNum = 2;//状态向量维数
    const int MeasureNum = 1;//观测向量维数
    uint32_t last_t = 0;
    float dt = 0.01;//时间间隔

public:
    /*参数拟合值*/
    double a = 0.9;// 振幅 [0.780, 1.045]
    double w = 1.9;// 频率 [1.884, 2.000]
    double t0 = 0.0;// 初相
    /**
     *  @brief  初始化状态值
     *  @param  angle_speed_time   大符速度和线形插值后的时间戳，在这里只用到speed和time
     */
    void initState(AngleSpeedTime angle_speed_time);
    
    /**
     *  @brief  预测
     *  @param  angle_speed_time   大符速度和线形插值后的时间戳
     */
    AngleSpeedTime predict(AngleSpeedTime angle_speed_time);

    SpdPredictor();
    ~SpdPredictor() = default;
};

#endif;