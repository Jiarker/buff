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
<<<<<<< HEAD
    cv::Point2f relative_point = point - org;                                         // ç›¸å¯¹åæ ‡
    double relative_angle = atan2(relative_point.y, relative_point.x);                // ä¸åœ†å¿ƒæ‰€æˆè§’åº¦
    double next_angle;

    if (is_clockwise) // é¡ºæ—¶é’ˆè¿åŠ¨
=======
    cv::Point2f relative_point = point - org;                                         // Ïà¶Ô×ø±ê
    double relative_angle = atan2(relative_point.y, relative_point.x);                // ÓëÔ²ĞÄËù³É½Ç¶È
    double next_angle;

    if (is_clockwise) // Ë³Ê±ÕëÔË¶¯
>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747
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
<<<<<<< HEAD
    SpeedTime flag_data = fitting_data[fitting_data.size() - 1];//æ¬¡æœ€æ–°æ•°æ®
    // if ((double)new_data.time - (double)flag_data.time - DT * 1000.0 < 0)//ä¸¤å¸§ä¹‹é—´æ—¶é—´é—´éš”å°äºé‡‡æ ·æ—¶é—´
=======
    SpeedTime flag_data = fitting_data[fitting_data.size() - 1];//´Î×îĞÂÊı¾İ
    // if ((double)new_data.time - (double)flag_data.time - DT * 1000.0 < 0)//Á½Ö¡Ö®¼äÊ±¼ä¼ä¸ôĞ¡ÓÚ²ÉÑùÊ±¼ä
>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747
    // {
    //     return;
    // }

    double T = 1000 * DT;
    double n = ((double)new_data.time - (double)flag_data.time) / T;

<<<<<<< HEAD
    if (n > 50)//æ‰‡å¶ä¸¢å¤±
=======
    if (n > 50)//ÉÈÒ¶¶ªÊ§
>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747
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
<<<<<<< HEAD
    return angle_diff / time_diff; // è½¬æ¢å•ä½
}


/*---------------æ‹Ÿåˆå‡½æ•°-----------------*/
=======
    return angle_diff / time_diff; // ×ª»»µ¥Î»
}


/*---------------ÄâºÏº¯Êı-----------------*/
// void FitTool::fittingCurve()
// {
//     if (fitting_data.empty())   
//         return;

//     if (fitting_data.size() >= N )
//     {
//         //¹¹½¨×îĞ¡¶ş³ËÎÊÌâ
//         ceres::Problem problem;
//         for(int i = 0; i < N; i++)
//         {
//             //¹¹ÔìÊ±¼ä±äÁ¿_(x)
//             double time = (double)(fitting_data[i].time - fitting_data[0].time);
//             //ÏòÎÊÌâÖĞÌí¼ÓÎó²îÏî
//             problem.AddResidualBlock(
//                 //Ê¹ÓÃ×Ô¶¯Çóµ¼,Ä£°å²ÎÊı:Îó²îÀàĞÍ¡¢Êä³öÎ¬¶È¡¢ÊäÈëÎ¬¶È,Î¬ÊıÒªÓëÇ°ÃæstructÒ»ÖÂ
//                 new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
//                     new CURVE_FITTING_COST(time, fitting_data[i].angle_speed)
//                 ),
//                 nullptr, //ºËº¯Êı,ÕâÀï²»Ê¹ÓÃ,Îª¿Õ
//                 awt //´ı¹À¼Æº¯Êı
//             );
//         }

//         //ÅäÖÃÇó½âÆ÷
//         ceres::Solver::Options options;//ÕâÀïÓĞºÜ¶àÅäÖÃÏî¿ÉÌî
//         options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;//ÔöÁ¿·½³ÌÈçºÎÇó½â
//         options.minimizer_progress_to_stdout = true; //Êä³öµ½cout

//         ceres::Solver::Summary summary;//ÓÅ»¯ĞÅÏ¢
//         ceres::Solve(options, &problem, &summary);//¿ªÊ¼ÓÅ»¯
//         cout<<summary.BriefReport()<<endl;
//         cout<<"estimated a, w, t = ";
//         for (auto a:awt)
//             cout<< a  <<" ";
//         cout << endl;
//         _a = awt[0];
//         _w = awt[1];
//         t_0 = awt[2];
//         is_Inited = true;
//     }
// }

>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747
void FitTool::fittingCurve()
{
    if (fitting_data.empty())   
        return;
<<<<<<< HEAD

    if (fitting_data.size() >= N )
    {
        //æ„å»ºæœ€å°äºŒä¹˜é—®é¢˜
        ceres::Problem problem;
        for(int i = 0; i < N; i++)
        {
            //æ„é€ æ—¶é—´å˜é‡_(x)
            double time = (double)(fitting_data[i].time - fitting_data[0].time);
            //å‘é—®é¢˜ä¸­æ·»åŠ è¯¯å·®é¡¹
            problem.AddResidualBlock(
                //ä½¿ç”¨è‡ªåŠ¨æ±‚å¯¼,æ¨¡æ¿å‚æ•°:è¯¯å·®ç±»å‹ã€è¾“å‡ºç»´åº¦ã€è¾“å…¥ç»´åº¦,ç»´æ•°è¦ä¸å‰é¢structä¸€è‡´
                new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                    new CURVE_FITTING_COST(time, fitting_data[i].angle_speed)
                ),
                nullptr, //æ ¸å‡½æ•°,è¿™é‡Œä¸ä½¿ç”¨,ä¸ºç©º
                awt //å¾…ä¼°è®¡å‡½æ•°
            );
        }

        //é…ç½®æ±‚è§£å™¨
        ceres::Solver::Options options;//è¿™é‡Œæœ‰å¾ˆå¤šé…ç½®é¡¹å¯å¡«
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;//å¢é‡æ–¹ç¨‹å¦‚ä½•æ±‚è§£
        options.minimizer_progress_to_stdout = true; //è¾“å‡ºåˆ°cout

        ceres::Solver::Summary summary;//ä¼˜åŒ–ä¿¡æ¯
        ceres::Solve(options, &problem, &summary);//å¼€å§‹ä¼˜åŒ–
        cout<<summary.BriefReport()<<endl;
        cout<<"estimated a, w, t = ";
        for (auto a:awt)
            cout<< a  <<" ";
        cout << endl;
        _a = awt[0];
        _w = awt[1];
        t_0 = awt[2];
=======
    else
    {
        //fitting_w();
        //fitting_a();
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
>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747
        is_Inited = true;
    }
}

<<<<<<< HEAD
=======
void FitTool::fitting_t()
{
    double max_value = 0.0, value = 0.0;
    int max_i = 0;
    MAX_T0 = 2 * M_PI / _w;
    for (int i = 0; i < T0_N + 1; i++)
    {
        value = get_integral((double)i * MAX_T0 / T0_N);
        cout<<"t0:"<<(double)i * MAX_T0 / T0_N<<"\tvalue:"<<value<<endl;
        if (value > max_value)
        {
            max_i = i;
            max_value = value;
        }
    }
    t_0 = (double)max_i * MAX_T0 / T0_N;
    start_time = fitting_data[0].time;
}

>>>>>>> 7db34b2a1dfdbee6ea161b361117d1f6fd771747

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