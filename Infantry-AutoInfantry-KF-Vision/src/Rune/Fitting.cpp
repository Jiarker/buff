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
            while (fitting_data_w.size() > 200)
                fitting_data_w.erase(fitting_data_w.begin());
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
    double relative_angle = atan2(relative_point.y, relative_point.x);                // 与圆心所成角庄1�7
    double next_angle;

    if (is_clockwise) // 顺时针运劄1�7
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
        if(armor_buffer.size() == 2 && !is_Kalman_init)
        {
            Kalman_init(armor_buffer[1], armor_buffer[0]);
            is_Kalman_init = true;
        }
        else
        {
            Kalman_predict(armor_1);
            //选出时间间隔在(5, 30)的角度并对其求速度
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
        }
        break;

    case ArmorState::LOST:
        armor_buffer.clear();
        fitting_data_w.clear();
        return false;
        break;
        
    default:
        return false;
        break;
    }
    while (fitting_data_w.size() > N)
        fitting_data_w.erase(fitting_data_w.begin());
    
    return true;
}

void FitTool::pushFittingData(SpeedTime new_data)
{
    //cout<<"Speed:"<<new_data.angle_speed<<"\ttime:"<<new_data.time<<endl;
    fitting_data.push_back(new_data);
    
    double temp_a = (2.090 - new_data.angle_speed) / 2; 
    //a范围[0.780,1.045] 
    if(temp_a >= 0.770 && temp_a <= 1.055)
    {
        est_a.push_back(temp_a);
    }
    if(est_a.size() > 80)
        est_a.erase(est_a.begin());


    if (fitting_data_w.empty())
    {
        fitting_data_w.push_back(new_data);
        return;
    }
    SpeedTime flag_data = fitting_data_w[fitting_data_w.size() - 1];//朢�新数捄1�7
    if ((double)new_data.time - (double)flag_data.time - DT * 1000.0 < 0)//两帧之间时间间隔小于采样时间
    {
        cout<<"None"<<endl;
        return;
    }

    double T = 1000 * DT;
    double n = ((double)new_data.time - (double)flag_data.time) / T;

    if (n > 50)
    {
        clearData();
        return;
    }

    for (int i = 0; i < (int)n; i++)//线��插倄1�7
    {
        //cout << "Intering: " << n << endl;
        // cout << "New: " << new_data.time << "\tOld: " << flag_data.time << endl;
        double temp_T = T * (i + 1);
        double delta = (double)new_data.time - (double)flag_data.time - temp_T;
        double temp_speed = new_data.angle_speed * (temp_T / (temp_T + delta)) + flag_data.angle_speed * (delta / (temp_T + delta));
        uint32_t temp_time = flag_data.time + (uint32_t)temp_T;
        SpeedTime temp = SpeedTime(temp_speed, temp_time);
        //SpeedTime temp = SpeedTime(new_data.angle_speed * (temp_T / (temp_T + delta)) + flag_data.angle_speed * (delta / (temp_T + delta)), flag_data.time + (uint32_t)temp_T);  
        fitting_data_w.push_back(temp); 
    }
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
    is_Kalman_init = false;
    angle_T = 0;
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
    if (fitting_data_w.empty() || fitting_data.empty())   return;
    if (fitting_data_w[fitting_data_w.size() - 1].time - fitting_data_w[0].time >= (N - 1) * DT * 1000 - 1)
    {
        fitting_w();
        fitting_a();
        if (isnan(_a))
        {
            cout << "A nan" << endl;
            _a = 0.9;
            _w = 1.9;
            t_0 = 0;
            clearData();
            return;
        }
        fitting_t();
        is_Inited = true;
    }
}

void FitTool::fitting_w()
{
    int n_min = 1.884 / (2 * M_PI) * N;
    int n_max = 2.0 / (2 * M_PI) * N + 1;

    double max_i = n_min;
    double max_value = get_F(n_min, N), value = 0.0;
    for (int i = n_min + 1; i < n_max; i++)
    {
        value = get_F(i, N);
        //cout<<"value:"<<value<<endl;
        if (value > max_value)
        {
            max_i = (double)i;
            max_value = value;
        }
    }
    _w = max_i / (double)N * 2.0 * M_PI;
    
    //_a = max_value / N * 2;
    // if (_a > 1.045) _a = 1.045;
    // else if (_a < 0.780) _a = 0.780;
    //_a = 0.780;
}

void FitTool::fitting_a()
{
    //按照大小划分占比
    double sum = 0.0;
    sort(est_a.begin(),est_a.end());
    //int sum_i = (0 + est_a.size() - 1) * est_a.size() / 2;
    //五次方超界，选择四次斄1�7
    long int four_sum_i = 0;
    for(int i = 0; i < est_a.size(); i++)
        four_sum_i += i * i * i * i ;

    for(int i = 0; i < est_a.size(); i++)
    {
        sum += est_a[i] * i * i * i * i  / four_sum_i;
    } 
    
    if(sum > 1.045)
        _a = 1.045;
    else if(sum < 0.780)
        _a = 0.780;
    else
        _a = sum;
}

void FitTool::fitting_t()
{
    double max_value = 0.0, value = 0.0;
    int max_i = 0;
    MAX_T0 = 2 * M_PI / _w;
    for (int i = 0; i < T0_N + 1; i++)
    {
        value = get_integral((double)i * MAX_T0 / T0_N);
        //cout<<"t0:"<<(double)i * MAX_T0 / T0_N<<"\tvalue:"<<value<<endl;
        if (value > max_value)
        {
            max_i = i;
            max_value = value;
        }
    }
    t_0 = (double)max_i * MAX_T0 / T0_N;
    start_time = fitting_data[0].time;
}

void FitTool::clearData()
{
    cout << "Clear Fitting Data!" << endl;
    fitting_data.clear();
    fitting_data_w.clear();
    armor_buffer.clear();
    est_a.clear();
    is_Kalman_init = false;
    angle_T = 0;
    is_direction_inited = false;
}

FitTool::FitTool(uint32_t _start_time)
{
    start_time = _start_time;
}

void FitTool::Kalman_predict(RuneArmor armor_new)
{
    if(armor_new.angle - last_angle_time.angle < -M_PI)//顺时针旋转引起角度突变
        angle_T += 1;
    else if(armor_new.angle - last_angle_time.angle > M_PI)//逆时针旋转引起角度突变
        angle_T -= 1;
    double correct_angle = spdpredictor.predict(armor_new.angle + 2*M_PI*angle_T, armor_new.gyro_pose.timestamp);
    armor_new.angle = correct_angle;
}

void FitTool::Kalman_init(RuneArmor armor_new, RuneArmor armor_old)
{
    if(armor_new.angle - armor_old.angle < -M_PI)//顺时针旋转引起角度突变
        angle_T += 1;
    else if(armor_new.angle - armor_old.angle > M_PI)//逆时针旋转引起角度突变
        angle_T -= 1;
    double angle_init = armor_new.angle + 2*angle_T*M_PI + armor_old.angle;
    double speed_init = (armor_new.angle + 2*angle_T*M_PI - armor_old.angle)/((double)(armor_new.gyro_pose.timestamp - armor_old.gyro_pose.timestamp)/1000.0); 
    uint32_t time_init = (armor_new.gyro_pose.timestamp + armor_old.gyro_pose.timestamp) / 2;
    spdpredictor.initState(angle_init, speed_init, time_init);
    last_angle_time = AngleTime(angle_init, time_init);
};