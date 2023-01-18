/*-------------------------------------------------------------
brief  :  RuneDetector
author :  XDU Robomaster, Jiarker
date   :  2023.1.13
---------------------------------------------------------------*/

#ifndef RUNEDETECTOR_H
#define RUNEDETECTOR_H

#include "RuneBox.h"
#include "Solver.h"
#include "Mat_time.h"
#include "const.h"
#include "DebugParam.h"

class RuneDetector
{
    private:
    Mat_time src;
    Mat bin;

    RuneParam param;
    DebugParam debug_param;

    COLOR enemy_color;
    vector<RuneArmor> rune_armors;

    int lost_times = 0; // 丢失目标

public:
    RuneArmor target;
    RuneArmor last_target;
    vector<cv::Point2f> nextPos;

    ArmorState state = ArmorState::LOST;

    Solver solver;

public:
    RuneDetector();
    ~RuneDetector() = default;

    /**
     *  @brief  API
     *  @param  _src 输入的图像
     */
    bool run(Mat_time _src);

    //能量机关击打颜色为我方颜色
    void setEnemyColor(COLOR color)
    {
        if (color == COLOR::BLUE)
            enemy_color = COLOR::RED;
        else if (color == COLOR::RED)
            enemy_color = COLOR::BLUE;
    }

private:
    /**
     *  @brief  亮度阈值+颜色通道相减，预处理图像
     *  @param  frame 原图像
     *  @param  bin 二值图
     */
    void preDeal(Mat frame);

    /**
     *  @brief  找到装甲板并区分
     *  @param  bin 二值图
     */
    bool findRuneArmor();

    /**
     *   @brief  模板匹配筛选装甲板
     *   @param  in_vane    内扇叶
     *   @param  out_vane   外扇叶
     *   
    */
   void moduleCmpare(Vane in_vane, Vane out_vane);

    /**
     *  @brief  选择待激活目标
     *  @param  candidate 候选装甲板
     */
    bool chooseTarget(vector<RuneArmor> candidate);
    
    /**
     *  @brief  测算圆心R
     *  @param  装甲板目标
     */
    bool calCenter(RuneArmor &t_armor);

    bool ifOldArmor();

    /**
     *  @brief  找到装甲板后的跟踪状态设置
     */
    void setFoundState()
    {
        lost_times = 0;

        switch (state)
        {
        case ArmorState::LOST:
        case ArmorState::FINDING:
            state = ArmorState::FIRST;
            break;
        case ArmorState::FIRST:
        case ArmorState::SHOOT:
            if (ifOldArmor())
            {
                state = ArmorState::SHOOT;
            }
            else
                state = ArmorState::FIRST;
            break;
        default:
            break;
        }
    }
};


#endif