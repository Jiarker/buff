#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//spd = a * sin[w * (t + t0)] + (2.090 - a)
//积分后：angle = -(a / w) * cons[w * (t + t0)] + (2.090 - a) * t + (a / w) * cos(w * t)
//w:[1.884,2.000]
//T:[3.1415,3.3350]
//b = 2.090 - a
//a:[0.780,1.045]

//曲线拟合所用时间
//4s = 400 * 10ms

//ext_a()所需数量为75~80,正好为一周期

int main()
{
    FitTool fittool;
    cv::RNG rng;
    double data_angle;
    uint32_t data_time = 0;

    double a = 0.9;
    double b = 2.090 - a;
    double w = 1.884;
    double t0 = 1.8;

    uint32_t prevent_time = 0;
    double prevent_angle = 0;

    uint32_t KF_prevent_time = 0;
    double KF_prevent_angle = 0;

    double S = 0;
    double S_KF = 0;

    //产生随机数
    srand(cv::getTickCount());
    a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    b = 2.090 - a;
    w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;

    double sum = 420;//fitting_data_w.size()超过400才能进行拟合

    for(int i = 0; i < sum; i++)
    {
        data_time += (20 + rng.gaussian(1));
        data_angle = (-a/w)*cos(w*((float)data_time/1000.0 + t0)) + (float)data_time/1000.0*(2.090-a) + (a/w)*cos(w*t0);
        //time += 10;
        if(i == 0)
        {
                prevent_angle = data_angle;
                prevent_time = data_time;
                KF_prevent_angle = data_angle;
                KF_prevent_time = data_time;

        }
        else if(i == 1)
        {
                uint32_t time = (data_time + prevent_time) / 2;
                double speed = (data_angle - prevent_angle) / ((double)(data_time - prevent_time) / 1000);
                double angle = (data_angle + prevent_angle) / 2;
                fittool.spdpredictor.initState(angle, speed, time);

        }
        else
        {
                //求出Speed对应时间戳与正确的速度
                uint32_t time = (data_time + prevent_time) / 2;
                double true_speed = a * sin(w * ((float)time/1000.0 + t0)) + (2.090 - a);
                
                //求出常规方法计算出的时间与方差
                double normal_speed = (data_angle - prevent_angle) / ((double)(data_time - prevent_time) / 1000);
                S += (true_speed - normal_speed)*(true_speed - normal_speed);
                
                //求出经卡尔曼滤波处理后的角度、用其计算的速度与方差
                double angle = fittool.spdpredictor.predict(data_angle, data_time);
                double speed = (angle - KF_prevent_angle) / ((double)(data_time - KF_prevent_time) / 1000);
                S_KF += (true_speed - speed)*(true_speed - speed);
                
                fittool.pushFittingData(SpeedTime(speed, time));

                cout<<"true_speed:"<<true_speed<<"\tnormal_speed:"<<normal_speed<<"\tspeed:"<<speed<<endl;
                //更新
                prevent_angle = angle;
                prevent_time = data_time;
                KF_prevent_angle = data_angle;
                KF_prevent_time = data_time;
        }
        //fittool.pushFittingData(SpeedTime(data, time));
        //cout<<"speed:"<<data<<"\ttime:"<<time<<endl;
    }

    
    // for (int i = 0; i < fittool.fitting_data_w.size(); i++)
    //     cout <<"The angle_speed in "<<fittool.fitting_data_w[i].time<<":"<< fittool.fitting_data_w[i].angle_speed << endl;
    cout<<"The range of est_a:"<<fittool.est_a.size()<<endl;
    cout <<"The range of fitting_data_w:"<<fittool.fitting_data_w.size() << endl;
    cout<<"The rangel of fitting_data:"<<fittool.fitting_data.size()<<endl;
    double t = cv::getTickCount();
    fittool.initDirection();
    fittool.fittingCurve();
    cout << "Time: " << (cv::getTickCount() - t) / cv::getTickFrequency() << endl;
    cout<<"S:"<<S<<"\tS_KF:"<<S_KF<<"\tS-S_KF :"<<S - S_KF<<endl;
    fittool.printResult_demo(a,w,t0);
}