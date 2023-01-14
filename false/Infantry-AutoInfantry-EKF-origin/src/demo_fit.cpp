#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

//角函数呈周期性变化。速度目标函数为：spd = a*sin(w(t+t0)) + b，其中 spd 的单位为rad/s
//w:[1.884,2.000]
//T:[3.1415,3.3350]
//b = 2.090 - a
//a:[0.780,1.045]

//拟合时间：
//假设1s发弹1次
//小符预击打：2s = 200 * 10ms
//大符预击打：4s = 400 * 10ms
//大符最大总击打时间：4 + 6 + 6 = 16s = 1600 * 10ms

//观察发现,一个周期能采集到的est_a数据大小为75~80,故令est_a大小为80

int main()
{

    FitTool fittool;
    SpdPredictor spdpredictor;
    cv::RNG rng;
    double data;
    uint32_t time = 0;

    double a = 0.903;
    double b = 2.090 - a;
    double w = 1.950;
    double t0 = 1.2;
    double sum = 500;//采样数量;fitting_data.size()超过400开始拟合

    //随机初始化参数
    srand(cv::getTickCount());
    a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    b = 2.090 - a;
    w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;

    vector<AngleSpeedTime>angle_speed_times;
    double last_angle = 0;
    uint32_t last_time = 0;
    for(int i = 0; i < sum; i++)
    {
        double data_angle = (-a/w)*cos(w*((float)time/1000.0 + t0)) + (float)time/1000.0*(2.090-a) + (a/w)*cos(w*t0);
        angle_speed_times.push_back(AngleSpeedTime(data_angle,0,time));
        //cout<<"time:"<<time<<"\tdata_angle:"<<data_angle<<endl;
        if(i == 0)//首次接收到数据,不进行初始化,记录数据
        {
            //spdpredictor.initState(AngleSpeedTime(data_angle, time));
            last_angle = data_angle;
            last_time = time; 
        }
        else if(i == 1)//第二次接收到数据,计算角速度,初始化卡尔曼滤波器
        {
            spdpredictor.initState(AngleSpeedTime(((data_angle + last_angle)/2), ((data_angle - last_angle)/(time - last_time)), (uint32_t)(((double)time + (double)last_time)/2)));
            angle_speed_times.push_back(AngleSpeedTime(((data_angle + last_angle)/2), ((data_angle - last_angle)/(time - last_time)), (uint32_t)(((double)time + (double)last_time)/2)));
        }
        else
        {
            //cout<<"time:"<<time<<endl;
            AngleSpeedTime correct_angle_speed_time = spdpredictor.predict(AngleSpeedTime(data_angle, 0, time));//预测
            //cout<<angle_speed_times.size()<<endl;
            //cout<<fittool.correct_angle_speed_time.angle<<endl;
            while(angle_speed_times.size() > 2)
            {
                uint32_t delta_time = correct_angle_speed_time.time - angle_speed_times[0].time;
                if(delta_time > 30)
                    angle_speed_times.erase(angle_speed_times.begin());
                else if(delta_time > 5)
                {
                    //cout<<"time:"<<spdpredictor.predict(AngleSpeedTime(data_angle,0, time)).time<<"\tspeed:"<<spdpredictor.predict(AngleSpeedTime(data_angle,0, time)).speed<<endl;
                    fittool.pushFittingData(correct_angle_speed_time);
                    break;
                }
                else
                    break;
            }
        }
        time += (10) + rng.gaussian(1);//每10ms进行一次采样
    }

    // for (int i = 0; i < fittool.fitting_data.size(); i++)
    //     cout <<"The angle_speed in "<<fittool.fitting_data[i].time<<":"<< fittool.fitting_data[i].angle_speed << endl;
    cout<<"The range of fittool.est_a:"<<fittool.est_a.size()<<endl;
    cout <<"The range of fittool.fitting_data:"<<fittool.fitting_data.size() << endl;
    double t = cv::getTickCount();
    fittool.initDirection();
    fittool.fittingCurve();
    cout << "Time: " << (cv::getTickCount() - t) / cv::getTickFrequency() << endl;
    fittool.printResult_demo(a,w,t0);
    return 0;
}