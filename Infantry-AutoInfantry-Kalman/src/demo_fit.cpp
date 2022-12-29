#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

//�Ǻ����������Ա仯���ٶ�Ŀ�꺯��Ϊ��spd = a*sin(w(t+t0)) + b������ spd �ĵ�λΪrad/s
//w:[1.884,2.000]
//T:[3.1415,3.3350]
//b = 2.090 - a
//a:[0.780,1.045]

//���ʱ�䣺
//����1s����1��
//С��Ԥ����2s = 200 * 10ms
//���Ԥ����4s = 400 * 10ms
//�������ܻ���ʱ�䣺4 + 6 + 6 = 16s = 1600 * 10ms

//�۲췢��,һ�������ܲɼ�����est_a���ݴ�СΪ75~80,����est_a��СΪ80

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
    double sum = 500;//��������;fitting_data.size()����400��ʼ���

    //�����ʼ������
    srand(cv::getTickCount());
    a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    b = 2.090 - a;
    w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;

    // double sum_variance_t = 0.0;
    // //while(t0 < 3.0)
    // {
    //     for(int i = 0; i < sum; i++)
    //     {
    //         data = a * sin((i * 0.01 + t0) * w) + (2.090 - a);
    //         time += 10;
    //         //time += (10 + rng.gaussian(1));//ÿ10ms����һ�β���
    //         fittool.pushFittingData(SpeedTime(data, time));
    //     }
    //     fittool.initDirection();
    //     fittool.fittingCurve();
    //     cout<<"t0:"<<t0<<"\tderta_t:"<<t0 - fittool.t_0<<endl;
    //     cout<<endl;
    //     sum_variance_t += pow(w - fittool.t_0, 2);
    //     t0 += 0.01;
    // }
    // cout<<"sum_variance_t:"<<sum_variance_t<<endl;


    // for(int i = 0; i < sum; i++)
    // {
    //     data = a * sin((i * 0.01 + t0) * w) + (2.090 - a);
    //     //time += 10;
    //     time += (10 + rng.gaussian(1));//ÿ10ms����һ�β���
    //     fittool.pushFittingData(SpeedTime(data, time));
    //     //cout<<"data:"<<data<<endl;
    // }

    vector<AngleTime>angle_times;
    for(int i = 0; i < sum; i++)
    {
        double data_angle = (-a/w)*cos(w*((float)time/1000.0 + t0)) + (float)time/1000.0*(2.090-a) + (a/w)*cos(w*t0);
        //cout<<"time:"<<time<<"\tdata_angle:"<<data_angle<<endl;
        angle_times.push_back(AngleTime(data_angle, time));
        if(i == 0)
            spdpredictor.initState(AngleTime(data_angle, time));
        else
        {
            SpeedTime correct_speed_time = spdpredictor.predict(AngleTime(data_angle, time));//Ԥ��
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
        time += (10) + rng.gaussian(1);//ÿ10ms����һ�β���
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