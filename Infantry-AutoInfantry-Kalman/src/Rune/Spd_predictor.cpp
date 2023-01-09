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
    
    KF->processNoiseCov = processNoise;                        //Q ????
    KF->measurementNoiseCov = measurementNoise;                //R ????
    cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));     //P ?????
    //cv::setIdentity(KF->transitionMatrix, cv::Scalar::all(1)); // F 状态转移矩阵
    KF->transitionMatrix = F;                                  //F ??????
    KF->measurementMatrix = H;                                 //H ????

}

void SpdPredictor::initState(double angle, double speed, uint32_t time)
{
    cv::Mat statePost = (cv::Mat_<float>(StateNum, 1) << angle, speed);
    KF->statePost = statePost;
    std::cout<<"first_init_angle:"<<KF->statePost.at<float>(0,0)
            <<"\tfirst_init_speed:"<<KF->statePost.at<float>(1,0)
            <<"\tfirst_init_time:"<<time
            <<std::endl;
    last_t = time;
    //std::cout<<"init_angle:"<<angle<<"\tinit_speed:"<<speed<<"\tinit_time:"<<time<<std::endl;
}

SpeedTime SpdPredictor::predict(AngleTime angle_time)
{
        //std::cout<<"init_angle:"<<KF->statePost.at<float>(0,0)
        //    <<"\tinit_speed:"<<KF->statePost.at<float>(1,0)
        //    <<std::endl;
    //计算预测值
    dt = (double)(angle_time.time - last_t) / 1000.0;
    last_t = angle_time.time;
    KF->transitionMatrix.at<float>(0, 1) = dt;
    KF->predict();

    //添加校正值
    cv::Mat correct_state;
    cv::Mat measurement = (cv::Mat_<float>(MeasureNum, 1) << angle_time.angle);
    KF->correct(measurement);

    float correct_angle = KF->statePost.at<float>(0,0);
    float correct_speed = KF->statePost.at<float>(1,0);
    std::cout<<"last_t:"<<last_t<<"\tspeed:"<<correct_speed<<"\tangle:"<<correct_angle<<"\tdt:"<<dt<<std::endl;
    SpeedTime correct_speed_time(correct_speed, last_t);
    return correct_speed_time;
    
}