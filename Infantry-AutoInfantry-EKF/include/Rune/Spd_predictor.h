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
    std::shared_ptr<cv::KalmanFilter> KF;//����ָ��
    const int StateNum = 2;//״̬����ά��
    const int MeasureNum = 1;//�۲�����ά��
    uint32_t last_t = 0;
    float dt = 0.01;//ʱ����

public:
    /*�������ֵ*/
    double a = 0.9;// ��� [0.780, 1.045]
    double w = 1.9;// Ƶ�� [1.884, 2.000]
    double t0 = 0.0;// ����
    /**
     *  @brief  ��ʼ��״ֵ̬
     *  @param  angle_speed_time   ����ٶȺ����β�ֵ���ʱ�����������ֻ�õ�speed��time
     */
    void initState(AngleSpeedTime angle_speed_time);
    
    /**
     *  @brief  Ԥ��
     *  @param  angle_speed_time   ����ٶȺ����β�ֵ���ʱ���
     */
    AngleSpeedTime predict(AngleSpeedTime angle_speed_time);

    SpdPredictor();
    ~SpdPredictor() = default;
};

#endif;