#include "Rune/Fitting.h"

/*-----------FitTool-----------*/
bool FitTool::run(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, ArmorState armor_state, Mode rune_mode)
{
    switch (rune_mode){
        case Mode::NORMAL_RUNE:
            return runNormalRune(armor_1, nextPosition, armor_state);
            
        case Mode::RUNE:
            return runRune(armor_1, nextPosition, armor_state);
        default:
            return false;
    }

}

bool FitTool::runRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, ArmorState armor_state)
{
    if (!processDataState(armor_1, armor_state))
        return false;
    initDirection();

    if (is_direction_inited)
    {
        fittingCurve();
        if (is_Inited)
        {
            nextPosition.clear();
            vector<cv::Point2f> pts;
            armor_1.getPoints(pts);
            double delta = deltaAngle(armor_1.gyro_pose.timestamp);

            for (int i = 0; i < 4; i++)
            {
                nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));
            }
            return true;
        }
        else
        {
            nextPosition.clear();
            vector<cv::Point2f> pts;
            armor_1.getPoints(pts);
            double delta = CV_PI / 3 * DELAY_TIME;
            for (int i = 0; i < 4; i++)
            {
                nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));
            }
            return true; 
        }
    }
    return false;
}

bool FitTool::runNormalRune(RuneArmor armor_1, vector<cv::Point2f> &nextPosition, ArmorState armor_state)
{
    switch (armor_state)
    {
    case ArmorState::FIRST:
        clearArmorBuffer();
        if (armor_1.gyro_pose.timestamp <= 10)
            return false;
        armor_buffer.push_back(armor_1);
        break;
    case ArmorState::SHOOT:
        if (armor_1.gyro_pose.timestamp <= 10)
            return false;
        armor_buffer.push_back(armor_1);
        if (armor_buffer.size() >= DN + 1)
        {
            pushFittingData(SpeedTime(calAngleSpeed(armor_1, armor_buffer[armor_buffer.size() - 1 - DN]), 
                                        (armor_1.gyro_pose.timestamp + armor_buffer[armor_buffer.size() - 1 - DN].gyro_pose.timestamp) / 2));
            while (armor_buffer.size() > DN + 1)
                armor_buffer.erase(armor_buffer.begin());
            while (fitting_data.size() > 200)
                fitting_data.erase(fitting_data.begin());
        }
        break;
    case ArmorState::LOST:
        clearData();
        return false;
    default:
        return false;
        break;
    }
    initDirection();

    if (is_direction_inited)
    {
        nextPosition.clear();
        vector<cv::Point2f> pts;
        armor_1.getPoints(pts);
        double delta = CV_PI / 3 * DELAY_TIME;
        for (int i = 0; i < 4; i++)
        {
            nextPosition.push_back(calNextPosition(pts[i], armor_1.circle_center, delta));
        }
        return true; 
    }
    return false;
}

cv::Point2f FitTool::calNextPosition(cv::Point2f point, cv::Point2f org, float rotate_angle)
{
    double radius = calDistance(point, org);
    cv::Point2f relative_point = point - org;                                         // 相对坐标
    double relative_angle = atan2(relative_point.y, relative_point.x);                // 与圆心所成角度
    double next_angle;

    if (is_clockwise) // 顺时针运动
    {
        next_angle = relative_angle + rotate_angle;
        if (next_angle > CV_PI)
            next_angle -= 2.0 * CV_PI;
    }
    else
    {
        next_angle = relative_angle - rotate_angle;
        if (next_angle < - CV_PI)
            next_angle += 2.0 * CV_PI;
    }

    return cv::Point2f(cos(next_angle) * radius, sin(next_angle) * radius) + org;
}

double FitTool::deltaAngle(uint32_t time)
{
    double t = (double)(time - start_time) / 1000.0;
    
    return (-_a / _w) * (cos(_w * (t + DELAY_TIME + t_0)) - cos(_w * (t + t_0))) + (2.090 - _a) * DELAY_TIME;
}

bool FitTool::processDataState(RuneArmor armor_1, ArmorState armor_state)
{
    switch (armor_state)
    {
    case ArmorState::FIRST:
        clearArmorBuffer();
        if (armor_1.gyro_pose.timestamp <= 10)
            return false;
        armor_buffer.push_back(armor_1);
        break;

    case ArmorState::SHOOT:
        if (armor_1.gyro_pose.timestamp <= 10)
            return false;

        armor_buffer.push_back(armor_1);
        while (armor_buffer.size() > 1 + DN)
        {
            uint32_t delta_time = armor_1.gyro_pose.timestamp - armor_buffer[0].gyro_pose.timestamp;
            if (delta_time > 30)
                armor_buffer.erase(armor_buffer.begin());
            else if (delta_time > 5)
            {
                pushFittingData(SpeedTime(calAngleSpeed(armor_1, armor_buffer[0]), 
                                            (armor_1.gyro_pose.timestamp + armor_buffer[0].gyro_pose.timestamp) / 2));
                break;
            }
            else
                break;
        }

        // if (armor_buffer.size() < DN + 1)
        // {
        //     armor_buffer.push_back(armor_1);
        // }
        // else if (armor_1.gyro_pose.timestamp - armor_buffer[armor_buffer.size() - 1 - DN].gyro_pose.timestamp > 5)
        // {
        //     armor_buffer.push_back(armor_1);
        //     pushFittingData(SpeedTime(calAngleSpeed(armor_1, armor_buffer[armor_buffer.size() - 1 - DN]), 
        //                                     (armor_1.gyro_pose.timestamp + armor_buffer[armor_buffer.size() - 1 - DN].gyro_pose.timestamp) / 2));

        //     while (armor_buffer.size() > DN + 1)
        //         armor_buffer.erase(armor_buffer.begin());
        // }
        break;

    case ArmorState::LOST:
        armor_buffer.clear();
        fitting_data.clear();
        return false;
        break;
        
    default:
        return false;
        break;
    }
    while (fitting_data.size() > N)
        fitting_data.erase(fitting_data.begin());
    
    return true;
}

void FitTool::pushFittingData(SpeedTime new_data)
{
    if (fitting_data.empty())
    {
        fitting_data.push_back(new_data);
        return;
    }
    SpeedTime flag_data = fitting_data[fitting_data.size() - 1];//次最新数据
    // if ((double)new_data.time - (double)flag_data.time - DT * 1000.0 < 0)//两帧之间时间间隔小于采样时间
    // {
    //     return;
    // }

    double T = 1000 * DT;
    double n = ((double)new_data.time - (double)flag_data.time) / T;

    if (n > 50)//扇叶丢失
    {
        clearData();
        return;
    }
    
    fitting_data.push_back(new_data);
}

void FitTool::initDirection()
{
    if (fitting_data.size() >= 50)
    {
        int clock = 0, clock_inv = 0;
        for (int i = 0; i < fitting_data.size(); i++)
        {
            if (fitting_data[i].angle_speed > 0)
                clock++;
            else
                clock_inv++;
        }
        is_direction_inited = true;
        is_clockwise = clock > clock_inv;
    }
}

void FitTool::clearArmorBuffer()
{
    armor_buffer.clear();
}

double FitTool::calAngleSpeed(RuneArmor armor_1, RuneArmor armor_2)
{
    double time_diff = (double)(armor_1.gyro_pose.timestamp - armor_2.gyro_pose.timestamp) / 1000.0;
    if (time_diff < 0.000001)
        time_diff += 0.005;

    double angle_diff = armor_1.angle - armor_2.angle;
    if (armor_1.angle < -CV_PI / 2.0 && armor_2.angle > CV_PI / 2.0)
        angle_diff = angle_diff + CV_PI * 2.0;
    else if (armor_1.angle > CV_PI / 2.0 && armor_2.angle < -CV_PI / 2.0)
        angle_diff = angle_diff - CV_PI * 2.0;
    return angle_diff / time_diff; // 转换单位
}


/*---------------拟合函数-----------------*/
void FitTool::fittingCurve()
{
    if (fitting_data.empty())   
        return;

    if (fitting_data.size() >= N )
    {
        //构建最小二乘问题
        ceres::Problem problem;
        for(int i = 0; i < N; i++)
        {
            //构造时间变量_(x)
            double time = (double)(fitting_data[i].time - fitting_data[0].time);
            //向问题中添加误差项
            problem.AddResidualBlock(
                //使用自动求导,模板参数:误差类型、输出维度、输入维度,维数要与前面struct一致
                new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                    new CURVE_FITTING_COST(time, fitting_data[i].angle_speed)
                ),
                nullptr, //核函数,这里不使用,为空
                awt //待估计函数
            );
        }

        //配置求解器
        ceres::Solver::Options options;//这里有很多配置项可填
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;//增量方程如何求解
        options.minimizer_progress_to_stdout = true; //输出到cout

        ceres::Solver::Summary summary;//优化信息
        ceres::Solve(options, &problem, &summary);//开始优化
        cout<<summary.BriefReport()<<endl;
        cout<<"estimated a, w, t = ";
        for (auto a:awt)
            cout<< a  <<" ";
        cout << endl;
        _a = awt[0];
        _w = awt[1];
        t_0 = awt[2];
        is_Inited = true;
    }
}


void FitTool::clearData()
{
    cout << "Clear Fitting Data!" << endl;
    fitting_data.clear();
    armor_buffer.clear();
    est_a.clear();
}

FitTool::FitTool(uint32_t _start_time)
{
    start_time = _start_time;
}