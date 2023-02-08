/* --------------------------------------------
 * Brief:   卡尔曼滤波
 * Author:  Parker   XDU IRobot
 * Date:    2022.12.28
 * -------------------------------------------- */

#ifndef SPD_PREDICTOR_H
#define SPD_PREDICTOR_H

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "Tools.h"
#include "Mat_time.h"
#include "AdaptiveEKF.hpp"


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

// typedef struct AngleTime
// {
//     double angle;
//     uint32_t time;

//     AngleTime(double a = 0.0, uint32_t t = 0)
//     {
//         angle = a;
//         time = t;
//     }
// }AngleTime;

class KFPredictor
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

    KFPredictor();
    ~KFPredictor() = default;
};


struct Predict
{
    template <class T>
    void operator()(const T x0[2], T x1[2], double delta_t, double a, double w)
    {
        x1[0] = x0[0] + (2/w)*(x0[1]-2.090+a)*sin(w*delta_t/2) + (2.090-a)*delta_t;
        x1[1] = x0[1];
    }
};

struct Measure
{
    template <class T>
    void operator()(const T observe[1], T y[1])
    {
        y[0] = observe[0];
    }
};

class EKFPredictor
{
private:
    /*data*/
    AdaptiveEKF<2, 1>*kalman_filter;
    bool is_kalman_init = false;
    void rebootKalman(double angle);
    void setUpdateTime(const double &delta_t);
    double correct(double angle);

    double update_time;
    //fitting解算参数
    double a;
    double w;

    uint32_t prevent_time;

    Eigen::Matrix<double, 2, 1>process_noice;
    Eigen::Matrix<double, 1, 1> process_noise_matrix;

    Predict predict_fun;
    Measure measure_fun;

public:
    EKFPredictor();
    ~EKFPredictor();
    double runKalman(const double angle = 0, uint32_t time = 0, double _a = 0, double _w = 0);
    void resetKalman()
    {
        is_kalman_init = false;
    };
};



#endif