/**
 * (a * sin(w * (x_ + t)) + 2.090 - a)
 */

#ifndef FITTING_H
#define FITTING_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "RuneBox.h"
#include "const.h"
#include "rm_types.h"
#include "Spd_predictor.h"

using namespace std;
using namespace cv;

class FitTool
{
public:
    vector<RuneArmor> armor_buffer;
    vector<SpeedTime> fitting_data_w;
    vector<SpeedTime> fitting_data;
    vector<double> est_a;
    SpdPredictor spdpredictor;//卡尔曼滤波器

private:
    double _a = 0.9;  // 振幅 [0.780, 1.045]
    double _w = 1.9;  // 频率 [1.884, 2.000]
    double t_0 = 0.0; // 初相

    double MAX_T0 = 3.34;  // 朢�大周朄1�7
    double T0_N = 90;      // 相位采样敄1�7
    double DT = 0.01;      // 采样时间间隔，单位：秄1�7
    double N = 400;        // 角��度采样敄1�7
    double DELAY_TIME = 0.37; // 预测时间，单位：秄1�7
    // double DELAY_TIME = 0;
    int DN = 1;            // 逐差法测速度间距

    uint32_t start_time;
    bool is_Inited = false;             // 大符拟合是否初始匄1�7
    bool is_direction_inited = false;   // 能量机关旋转方向初始匄1�7
    bool is_clockwise;     // 顺时钄1�7

    int angle_T = 0;        		// 角度转过的周期，用以处理转动角度。
    float last_angle = 0;   //上一个预测后的角度
    bool is_Kalman_init = false;	//卡尔曼滤波初始化

public:
    /**
     *  @brief  封装API
     */
    bool run(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, ArmorState armor_state, Mode rune_mode=Mode::RUNE);


    /**
     *  @brief  清空数据
     */
    void clearData();

    /**
     *  @brief  计算瞬时角��度 (armor_1 - armor_2)
     *  @param  armor_1 新目栄1�7
     *  @param  armor_2 老目栄1�7
     *  @return 角��度，单佄1�7 弧度/秄1�7
     */
    double calAngleSpeed(RuneArmor armor_1, RuneArmor armor_2);

    /**
     *  @brief  清空armor_buffer
     */
    void clearArmorBuffer();

    /**
     *  @brief  根据状��处理数捄1�7
     *  @param  armor_1 处理完的装甲杄1�7
     *  @param  timestamp   原图像时间戳
     */
    bool processDataState(RuneArmor armor_1, ArmorState armor_state);

    /**
     *  @brief  数据作插值处琄1�7
     */
    void pushFittingData(SpeedTime);

    /**
     *  @brief  初步拟合数据
     */
    void fittingCurve();

    /**
     *  @brief  判断能量机关旋转方向
     */
    void initDirection();

//private:
public:
    /**
     *  @brief  击打大符模式
     */
    bool runRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, ArmorState armor_state);

    /**
     *  @brief  击打小符模式
     */ 
    bool runNormalRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, ArmorState armor_state);

    /**
     *  @brief  通过离散傅里叶变换求w
     */
    void fitting_w();
    
    /**
     *  @brief  拟合振幅
     */
    void fitting_a();

    /**
     *  @brief  拟合相位
     */
    void fitting_t();

    /**
     *  @brief  预测角度
     *  @param  armor_1 新装甲板
     *  @return 返回角度
     */
    float predictAngle(RuneArmor armor_1);

    /**
     *  @brief  根据旋转角度和半径计算下丢�炄1�7(装甲板四个角炄1�7)的像素位罄1�7
     *  @param  point   动点
     *  @param  org     原点
     *  @param  angle   旋转角度
     */
    cv::Point2f calNextPosition(cv::Point2f point, cv::Point2f org, float angle);

    /**
     *  @brief  速度函数积分计算偏移规1�7
     *  @param  time    装甲板时间戳
     *  @return 返回增大的角度，单位：弧庄1�7
     */
    double deltaAngle(uint32_t time);

    /**
     *  @brief  大符公式，用于补帧处琄1�7
     *  @param  timestamp   时间戄1�7
     *  @return 角��度
     */
    double runeMotion(uint32_t timestamp)
    {
        return _a * sin(_w * ((double)(timestamp - start_time) + t_0)) + 2.09 - _a;
    }

    /**
     *  @brief  离散傅里叶获得正弦项倄1�7
     */
    double get_F_s(int n, double f_k, int k, int _N)
    {
        return f_k * sin(2.0 * M_PI * (double)n / (double)_N * (double)k * DT);
    }

    /**
     *  @brief  离散傅里叶获得余弦项倄1�7
     */
    double get_F_c(int n, double f_k, int k, int _N)
    {
        return f_k * cos(2.0 * M_PI * (double)n / (double)_N * (double)k * DT);
    }

    /**
     *  @brief 离散傅里叶获得第n项的值，规整化��度倄1�7
     *  @return 模的平方
     */
    double get_F(int n, int _N)
    {
        double c = 0.0, s = 0.0;
        if (is_clockwise)
            for (int i = 0; i < fitting_data_w.size(); i++)
            {
                c += get_F_c(n, (fitting_data_w[i].angle_speed - (2.090 - _a)), i, N);
                s += get_F_s(n, (fitting_data_w[i].angle_speed - (2.090 - _a)), i, N);
            }
        else
            for (int i = 0; i < fitting_data_w.size(); i++)
            {
                c += get_F_c(n, (-fitting_data_w[i].angle_speed - (2.090 - _a)), i, N);
                s += get_F_s(n, (-fitting_data_w[i].angle_speed - (2.090 - _a)), i, N);
            }

        return sqrt(c * c + s * s);
    }

    /**
     *  @brief  求不同相位时的积刄1�7,规整化��度倄1�7
     */
    double get_integral(double t_)
    {
        double sum = 0;
        if (is_clockwise)
            for (int i = 0; i < fitting_data.size(); i++)
            {
                sum += sin((i * DT + t_) * _w) * (fitting_data[i].angle_speed - (2.090 - _a)) / _a;
            }
        else
            for (int i = 0; i < fitting_data.size(); i++)
            {
                sum += sin((i * DT + t_) * _w) * (-fitting_data[i].angle_speed - (2.090 - _a)) / _a;
            }

        return sum;
    }

public:
    FitTool(uint32_t _start_time = 0);
    /**
     *  @brief  打印结果
     */
    // void printResult()
    // {
    //     cout << "a: " << _a << "\tw: " << _w << endl;
    //     cout << "t: " << t_0 << endl;
    // }
        void printResult()
    {
        cout << "a: " << _a << endl;
        cout<< "w: " << _w << endl;
        cout << "t: " << t_0 << endl;
    }
    
    /**
     *  @brief  打印测试结果
     */
    void printResult_demo(double a,double w,double t)
    {
        cout<<"T:"<<2 * CV_PI / w <<endl;
        cout << "a: " << _a <<"\tderta_a:"<< a - _a << endl;
        cout<< "w: " << _w <<"\tderta_w:"<< w - _w << endl;
        cout << "t: " << t_0<<"\tderta_t:"<< t - t_0 <<"\tt_correct:"<<t<< endl;
    }

	/**
   *    @brief  卡尔曼滤波器初始化
	*   @param 	armor_new 新装甲板
   */
  	void Kalman_init(RuneArmor armor_new, RuneArmor armor_old);

	/**
	*   @brief	卡尔曼滤波,并将滤波结果返回给armor_new
	*	@param	 armor_new	新装甲板
	*/
	void Kalman_predict(RuneArmor armor_new);

    ~FitTool() = default;
};

#endif