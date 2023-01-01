#include"Spd_predictor.h"

SpdPredictor::SpdPredictor()
{
    cv::FileStorage fs("../Configure/RuneParam.xml", cv::FileStorage::READ);
    cv::Mat processNoise, measurementNoise;
    fs["kalman_spd_Q"] >> processNoise;
    fs["kalman_spd_R"] >> measurementNoise;
    fs.release();

    KF = std::make_shared<cv::KalmanFilter>(StateNum, MeasureNum, 0);

    cv::Mat H = (cv::Mat_<float>(MeasureNum, StateNum) << 1, 0);
    cv::Mat F = (cv::Mat_<float>(StateNum, StateNum) << 1, dt,
                                                        0, 1);
    
    KF->processNoiseCov = processNoise;                        //Q ¹ı³ÌÔëÉù
    KF->measurementNoiseCov = measurementNoise;                //R ²âÁ¿ÔëÉù
    cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));     //P ºóÑéÎó²î¹À¼ÆĞ­·½²î¾ØÕó
    KF->transitionMatrix = F;                                  //F ×´Ì¬×ªÒÆ¾ØÕó/¿ØÖÆ¾ØÕó
    KF->measurementMatrix = H;                                 //H ²âÁ¿¾ØÕó

}

void SpdPredictor::initState(AngleSpeedTime angle_speed_time)
{
    KF->statePost = (cv::Mat_<float>(StateNum,1) << angle_speed_time.angle,
                                                    angle_speed_time.speed);
    last_t = angle_speed_time.time;
    std::cout<<"last_time:"<<last_t<<std::endl;
}

AngleSpeedTime SpdPredictor::predict(AngleSpeedTime angle_speed_time)
{
    cv::Mat correct_state;
    cv::Mat measurement = (cv::Mat_<float>(MeasureNum, 1) << angle_speed_time.angle);
    dt = (float)(angle_speed_time.time - last_t) / 1000.0;
    KF->transitionMatrix.at<float>(0, 1) = dt;
    last_t = angle_speed_time.time;
    KF->predict();

    correct_state = KF->correct(measurement);//??
    float correct_angle = correct_state.at<float>(0,0);
    float correct_speed = correct_state.at<float>(1,0);
    //std::cout<<"correct_last_t:"<<last_t<<std::endl;
    //std::cout<<"last_t:"<<last_t<<"\tspeed:"<<correct_speed<<"\tangle:"<<correct_angle<<std::endl;
    return AngleSpeedTime(correct_angle, correct_speed, last_t);
    
}