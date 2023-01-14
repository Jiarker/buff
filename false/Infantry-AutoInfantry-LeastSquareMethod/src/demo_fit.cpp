#include "Rune/Fitting.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

<<<<<<< HEAD
//½Çº¯Êý³ÊÖÜÆÚÐÔ±ä»¯¡£ËÙ¶ÈÄ¿±êº¯ÊýÎª£ºspd = a * sin(w * (t + t0)) + (2.090 - a)£¬ÆäÖÐ spd µÄµ¥Î»Îªrad/s
=======
//ï¿½Çºï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô±ä»¯ï¿½ï¿½ï¿½Ù¶ï¿½Ä¿ï¿½êº¯ï¿½ï¿½Îªï¿½ï¿½spd = a * sin(w * (t + t0)) + (2.090 - a)ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ spd ï¿½Äµï¿½Î»Îªrad/s
>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747
//w:[1.884,2.000]
//T:[3.1415,3.3350]
//b = 2.090 - a
//a:[0.780,1.045]

<<<<<<< HEAD
//ÄâºÏÊ±¼ä£º
//¼ÙÉè1s·¢µ¯1´Î
//Ð¡·ûÔ¤»÷´ò£º2s = 200 * 10ms
//´ó·ûÔ¤»÷´ò£º4s = 400 * 10ms
//´ó·û×î´ó×Ü»÷´òÊ±¼ä£º4 + 6 + 6 = 16s = 1600 * 10ms

//¹Û²ì·¢ÏÖ,Ò»¸öÖÜÆÚÄÜ²É¼¯µ½µÄest_aÊý¾Ý´óÐ¡Îª75~80,¹ÊÁîest_a´óÐ¡Îª80
=======
//ï¿½ï¿½ï¿½Ê±ï¿½ä£º
//ï¿½ï¿½ï¿½ï¿½1sï¿½ï¿½ï¿½ï¿½1ï¿½ï¿½
//Ð¡ï¿½ï¿½Ô¤ï¿½ï¿½ï¿½ï¿½2s = 200 * 10ms
//ï¿½ï¿½ï¿½Ô¤ï¿½ï¿½ï¿½ï¿½4s = 400 * 10ms
//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ü»ï¿½ï¿½ï¿½Ê±ï¿½ä£º4 + 6 + 6 = 16s = 1600 * 10ms

//ï¿½Û²ì·¢ï¿½ï¿½,Ò»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ü²É¼ï¿½ï¿½ï¿½ï¿½ï¿½est_aï¿½ï¿½ï¿½Ý´ï¿½Ð¡Îª75~80,ï¿½ï¿½ï¿½ï¿½est_aï¿½ï¿½Ð¡Îª80
>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747

int main()
{
    FitTool fittool;
    cv::RNG rng;
    double data;
    uint32_t time = 2e7;

<<<<<<< HEAD
    double a = 1.045;
    double b = 2.090 - a;
    double w = 1.80;
    double t0 = 2.0;

    //Ëæ»ú³õÊ¼»¯²ÎÊý
    srand(cv::getTickCount());
    a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    b = 2.090 - a;
    w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;
    
    double sum = 700;//²ÉÑùÊýÁ¿;fitting_data.size()³¬¹ý400¿ªÊ¼ÄâºÏ
=======
    double a = 0.9;
    double b = 2.090 - a;
    double w = 1.87;
    double t0 = 2.0;

    //ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    srand(cv::getTickCount());
    // a = (rand() %(1045 - 780 + 1) + 780) * 1.0 / 1000.0;
    // b = 2.090 - a;
    // w = (rand() % (2000 - 1884 + 1) + 1884) * 1.0 / 1000.0;
    t0 = (rand() % (3000 - 0 + 1) + 0) * 1.0 / 1000.0;
    
    double sum = 700;//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½;fitting_data.size()ï¿½ï¿½ï¿½ï¿½400ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½
>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747

    for(int i = 0; i < sum; i++)
    {
        data = a * sin(w *( 0.01 * i + t0)) + (2.090 - a);
<<<<<<< HEAD
        time += (10 + rng.gaussian(1));//Ã¿10ms½øÐÐÒ»´Î²ÉÑù
=======
        time += (10 + rng.gaussian(1));//Ã¿10msï¿½ï¿½ï¿½ï¿½Ò»ï¿½Î²ï¿½ï¿½ï¿½
        //time += 10;
>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747
        fittool.pushFittingData(SpeedTime(data, time));
        cout<<"speed:"<<data<<"\ttime:"<<time<<endl;
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