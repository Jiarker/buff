/* --------------------------------------------
 * Brief:   ʹ�ýǶ�Ԥ����ٶȵĿ������˲���,
 * Author:  Parker   ����XDU IRobot
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
    std::shared_ptr<cv::KalmanFilter> KF;//����ָ��
    const int StateNum = 2;//״̬����ά��
    const int MeasureNum = 1;//�۲�����ά��
    uint32_t last_t = 0;
    float dt = 0.01;//ʱ����

public:
    /**
     *  @brief  ��ʼ��״ֵ̬
     *  @param  angle_time   ����ٶȺ����β�ֵ���ʱ���
     */
    void initState(double angle, double speed, uint32_t time);
    
    /**  
     *  @brief  Ԥ��
     *  @param  angle_time   ����ٶȺ����β�ֵ���ʱ���
     */
    SpeedTime predict(AngleTime angle_time);

    SpdPredictor();
    ~SpdPredictor() = default;
};

#endif