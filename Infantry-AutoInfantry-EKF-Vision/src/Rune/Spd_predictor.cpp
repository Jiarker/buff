#include"Spd_predictor.h"

KFPredictor::KFPredictor()
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
    //cv::setIdentity(KF->transitionMatrix, cv::Scalar::all(1)); // F ״̬ת�ƾ���
    KF->transitionMatrix = F;                                  //F ??????
    KF->measurementMatrix = H;                                 //H ????

}

void KFPredictor::initState(double angle, double speed, uint32_t time)
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

double KFPredictor::predict(double angle, uint32_t time)
{
        //std::cout<<"init_angle:"<<KF->statePost.at<float>(0,0)
        //    <<"\tinit_speed:"<<KF->statePost.at<float>(1,0)
        //    <<std::endl;
    //����Ԥ��ֵ
    dt = (double)(time - last_t) / 1000.0;
    last_t = time;
    KF->transitionMatrix.at<float>(0, 1) = dt;
    KF->predict();

    //����У��ֵ
    cv::Mat correct_state;
    cv::Mat measurement = (cv::Mat_<float>(MeasureNum, 1) << angle);
    KF->correct(measurement);

    float correct_angle = KF->statePost.at<float>(0,0);
    float correct_speed = KF->statePost.at<float>(1,0);
    //std::cout<<"last_t:"<<last_t<<"\tspeed:"<<correct_speed<<"\tangle:"<<correct_angle<<"\tdt:"<<dt<<std::endl;
    return correct_angle;   
}

//namespace ly

    EKFPredictor::EKFPredictor()
    {
        kalman_filter = new AdaptiveEKF<2, 1>();
        cv::FileStorage fs("../Configure/RuneParam.xml", cv::FileStorage::READ);
        cv::Mat processNoise, measurementNoise;
        fs["EKF_spd_Q"] >> processNoise;
        fs["EKF_spd_R"] >> measurementNoise;

        // cout<<kalman_filter->Q(0,0)<<endl;
        // kalman_filter->Q(0,0) = processNoise.at<double>(0,0),
        // kalman_filter->Q(1,1) = processNoise.at<double>(1,1);
        // kalman_filter->R(0,0) = measurementNoise.at<double>(0,0);
        // cout<<"-2"<<endl;

        //opencv与eigen矩阵转换
        cv::cv2eigen(processNoise, kalman_filter->Q);
        cv::cv2eigen(measurementNoise, kalman_filter->R);
        
        kalman_filter->P = Eigen::Matrix<double, 2, 2>::Identity();
        fs.release();
        is_kalman_init = false;
        kalman_filter = new AdaptiveEKF<2, 1>();
    }

EKFPredictor::~EKFPredictor(){}

void EKFPredictor::rebootKalman(double angle)
{
    kalman_filter->Xe[0] = angle;
    kalman_filter->Xe[1] = 0;
}

void EKFPredictor::setUpdateTime(const double &delta_t)
{
    if(fabs(delta_t) < 1e-4)//防止时间差为0
    {
        update_time = 8.0 / 1000.0;
    }
    else
    {
        update_time = delta_t / 1000.0;
    }
}

double EKFPredictor::correct(const double angle)
{
    kalman_filter->predict(predict_fun, update_time, a, w);
    Eigen::Matrix<double, 1, 1> measure;
    measure(0, 0) = angle;
    kalman_filter->update(measure_fun, measure);

    //预测结果
    double correct_angle = kalman_filter->Xe[0];
    double correct_speed = kalman_filter->Xe[1];
    //std::cout<<speed:"<<correct_speed<<"\tangle:"<<correct_angle<<std::endl;

    return correct_angle;
}


double EKFPredictor::runKalman(const double angle, uint32_t time, double _a, double _w)
{
    if(!is_kalman_init)
    {
        a = _a;
        w = _w;
        prevent_time = time;
        is_kalman_init = true;
        rebootKalman(angle);
        return angle;
    }
    else
    {
        // cout<<"a:"<<a<<"w:"<<w<<endl;
        double delta_t = double(time - prevent_time);
        prevent_time = time;
        setUpdateTime(delta_t);
        return correct(angle);
    }
}
