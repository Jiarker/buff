#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//角函数呈周期性变化。速度目标函数为：spd = a ∗ sin(𝜔 ∗ 𝑡) + 𝑏，其中 spd 的单位为rad/s
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
    // FitTool fittool;
    // ifstream fs("/home/shanzoom/PycharmProjects/TEst/data.txt");
    // cv::RNG rng;

    // double data;
    // uint32_t time = 2e7;
    // while (!fs.eof())
    // {
    //     fs >> data;
    //     time += (10 + rng.gaussian(1));//每10ms进行一次采样
    //     fittool.pushFittingData(SpeedTime(data, time));
    // }
    // fs.close();
    FitTool fittool;
    cv::RNG rng;
    double data;
    uint32_t time = 2e7;

    double a = 1.045;
    double b = 2.090 - a;
    double w = 1.80;
    double t0 = 2.0;

    double sum = 420;//采样数量;fitting_data.size()超过400开始拟合

    for(int i = 0; i < sum; i++)
    {
        data = a * sin(w * 0.01 * i + t0) + (2.090 - a);
        time += (10 + rng.gaussian(1));//每10ms进行一次采样
        fittool.pushFittingData(SpeedTime(data, time));
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
}