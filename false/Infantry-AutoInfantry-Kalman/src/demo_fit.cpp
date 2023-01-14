#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <stdlib.h>

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
    SpdPredictor spdpredictor;
    cv::RNG rng;
    double data;
    uint32_t time = 1e7;

    double a = 0.903;
    double b = 2.090 - a;
    double w = 1.950;
    double t0 = 1.2;
    double sum = 500;//fitting_data_w.size()超过400才能进行拟合
    
    //产生随机数
    srand(cv::getTickCount());
    a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    b = 2.090 - a;
    w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;

    vector<AngleTime>angle_times;
    uint32_t last_t;
    double last_angle;
    for(int i = 0; i < sum; i++)
    {
        double data_angle = (-a/w)*cos(w*((float)time/1000.0 + t0)) + (float)time/1000.0*(2.090-a) + (a/w)*cos(w*t0);
        //cout<<"time:"<<time<<"\tdata_angle:"<<data_angle<<endl;
        angle_times.push_back(AngleTime(data_angle, time));
        if(i == 0)
        {
            //spdpredictor.initState(AngleTime(data_angle, time));
            last_angle = data_angle;
            last_t = time;
        }
        else if(i == 1)
        {
            double init_angle = (data_angle + last_angle)/2;
            double speed = (data_angle - last_angle) / ((double)(time - last_t)/1000);
            uint32_t init_time = (time + last_t)/2;
            //double speed = a * sin(w * (time + t0)) + (2.090 - a);
            spdpredictor.initState(data_angle, speed, time);
        }
            
        else
        {
            // double angle = (data_angle + last_angle)/2;
            // double speed = (data_angle - last_angle) / ((double)(time - last_t)/1000);
            // uint32_t a_time = (time + last_t)/2;
            // cout<<"time:"<<a_time<<"\tangle:"<<angle<<"\tspeed:"<<speed<<endl;
            // last_angle = data_angle;
            // last_t = time;
            SpeedTime correct_speed_time = spdpredictor.predict(AngleTime(data_angle, time));
            while(angle_times.size() > 2)
            {
                uint32_t delta_time = correct_speed_time.time - angle_times[0].time;
                if(delta_time > 30)
                    angle_times.erase(angle_times.begin());
                else if(delta_time > 5)
                {
                    //cout<<"time:"<<spdpredictor.predict(AngleTime(data_angle, time)).time<<"\tspeed:"<<spdpredictor.predict(AngleTime(data_angle, time)).angle_speed<<endl;
                    fittool.pushFittingData(correct_speed_time);
                    break;
                }
                else
                    break;
            }
        }
        time += (10) + rng.gaussian(1);
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