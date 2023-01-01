#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//角函数呈周期性变化。速度目标函数为：spd = a * sin(w * (t + t0)) + (2.090 - a)，其中 spd 的单位为rad/s
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
    cv::RNG rng;
    double data;
    uint32_t time = 0;

    double a = 0.9;
    double b = 2.090 - a;
    double w = 1.9;
    double t0 = 2.0;

    //随机初始化参数
    srand(cv::getTickCount());
    a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    b = 2.090 - a;
    w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;
    
    double sum = 500;//采样数量;fitting_data.size()超过400开始拟合

    for(int i = 0; i < sum; i++)
    {
        //角度拟合曲线运动方程,此时的data实际上为angle
        data = -(a/w)*cos(w*((double)time/1000+t0)) + (2.090-a)*time/1000 + (a/w)*cos(w*t0);
        time += (10 + rng.gaussian(1));//每10ms进行一次采样
        fittool.pushFittingData(SpeedTime(data, time));
        cout<<"speed:"<<data<<"\ttime:"<<time<<endl;
    }

    
    // for (int i = 0; i < fittool.fitting_data.size(); i++)
    //     cout <<"The angle_speed in "<<fittool.fitting_data[i].time<<":"<< fittool.fitting_data[i].angle_speed << endl;
    cout<<"The range of fittool.est_a:"<<fittool.est_a.size()<<endl;
    cout <<"The range of fittool.fitting_data:"<<fittool.fitting_data.size() << endl;
    double t = cv::getTickCount();
    //fittool.initDirection();
    fittool.fittingCurve();
    cout << "Time: " << (cv::getTickCount() - t) / cv::getTickFrequency() << endl;
    fittool.printResult_demo(a,w,t0);
}