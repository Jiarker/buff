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
    double data;
    uint32_t time = 2e7;

    double a = 0.9;
    double b = 2.090 - a;
    double w = 1.884;
    double t0 = 1.8;

    //产生随机数
    srand(cv::getTickCount());
    a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    b = 2.090 - a;
    w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;

    double sum = 420;//fitting_data_w.size()超过400才能进行拟合

    for(int i = 0; i < sum; i++)
    {
        data = a * sin(w * (0.01 * i + t0)) + (2.090 - a);
        //time += 10;
        time += (10 + rng.gaussian(1));
        fittool.pushFittingData(SpeedTime(data, time));
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
    fittool.printResult_demo(a,w,t0);
}