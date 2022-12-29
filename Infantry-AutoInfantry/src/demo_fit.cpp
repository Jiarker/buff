#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//�Ǻ����������Ա仯���ٶ�Ŀ�꺯��Ϊ��spd = a �6�5 sin(�3�0 �6�5 �3�9) + �3�1������ spd �ĵ�λΪrad/s
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
    // FitTool fittool;
    // ifstream fs("/home/shanzoom/PycharmProjects/TEst/data.txt");
    // cv::RNG rng;

    // double data;
    // uint32_t time = 2e7;
    // while (!fs.eof())
    // {
    //     fs >> data;
    //     time += (10 + rng.gaussian(1));//ÿ10ms����һ�β���
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

    double sum = 420;//��������;fitting_data.size()����400��ʼ���

    for(int i = 0; i < sum; i++)
    {
        data = a * sin(w * 0.01 * i + t0) + (2.090 - a);
        time += (10 + rng.gaussian(1));//ÿ10ms����һ�β���
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