#ifndef RM_EKF_H
#define RM_EKF_H

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <opencv2/core.hpp>

template <int state_num, int measure_num, typename T = double>
class ExtendKalmanFilter
{
public:
    ExtendKalmanFilter() : j_transitionMatrix(Eigen::Matrix<T, state_num, state_num>::Identity()),
                           j_measuremenMatrix(Eigen::Matrix<T, measure_num, state_num>::Identity()),
                           measurementNoiseCov(Eigen::Matrix<T, measure_num, measure_num>::Identity()),
                           erroCovPost(Eigen::Matrix<T, state_num, state_num>::Identity()),
                           erroCovPre(Eigen::Matrix<T, state_num, state_num>::Identity()),
                           processNoiseCov(Eigen::Matrix<T, state_num, state_num>::Identity()),
                           gain(Eigen::Matrix<T, state_num, measure_num>::Zero()),
                           statePost(Eigen::Matrix<T, state_num, 1>::Zero()),
                           statePre(Eigen::Matrix<T, state_num, 1>::Zero())
    {
    }
    ~ExtendKalmanFilter() {}

    //预测结果
    template <class Func>
    Eigen::Matrix<T, state_num, 1> predict(double dt)
    {
        // set auto cal jet matrix for  statePose to statePre
        ceres::Jet<T, state_num> state_post_jet[state_num];
        for (int i = 0; i < state_num; i++)
        {
            state_post_jet[i].a = statePost[i];
            state_post_jet[i].v[i] = 1;
        }
        ceres::Jet<T, state_num> state_pre_jet[state_num];
        //predictFunc(state_post_jet, state_pre_jet);

        //方程更新数据
        state_pre_jet[0] = state_post_jet[0] + 2*(state_post_jet[1]-2.0290+_a)/_w*sin(_w*dt/2) + (2.090-a)/dt;
        state_pre_jet[1] = state_post_jet[1];

        for (int i = 0; i < state_num; i++)
        {
            // get statePre
            statePre[i] = state_pre_jet[i].a;
            // get the jet for A(transitionMatrix)
            j_transitionMatrix.block(i, 0, 1, state_num) = state_pre_jet[i].v.transpose();
        }
        // P= JA*P'*JA.trans()
        erroCovPre = j_transitionMatrix * erroCovPost * j_transitionMatrix.transpose() + processNoiseCov;
        return statePre;
    }

    //状态更新
    template <class Func>
    Eigen::Matrix<T, state_num, 1> correct(Eigen::Matrix<T, measure_num, 1> measure, double angle)
    {
        ceres::Jet<double, state_num> state_pre_jet[state_num];
        for (int i = 0; i < state_num; i++)
        {
            state_pre_jet[i].a = statePre[i];
            state_pre_jet[i].v[i] = 1;
        }
        ceres::Jet<double, state_num> state_mse_jet[measure_num];

        //measureFunc(state_pre_jet, state_mse_jet);
        state_mse_jet[0] = angle;

        //  cal h(x_pre)
        Eigen::Matrix<T, measure_num, 1> temp_msu; // h(statePre)
        for (int i = 0; i < measure_num; i++)
        {
            temp_msu[i] = state_mse_jet[i].a;
            j_measuremenMatrix.block(i, 0, 1, state_num) = state_mse_jet[i].v.transpose();
        }
        Eigen::Matrix<T, measure_num, measure_num> temp1 = (j_measuremenMatrix * erroCovPre * j_measuremenMatrix.transpose() + measurementNoiseCov).inverse();
        // K=P*JH*(JH*P*JH.trans() + R)^-1
        gain = erroCovPre * j_measuremenMatrix.transpose() * temp1;
        // X'= X+ K*(Z- H(X))
        statePost = statePre + gain * (measure - temp_msu);

        Eigen::Matrix<T, state_num, state_num> I = Eigen::Matrix<T, state_num, state_num>::Identity();
        // P' = (I-K*H)*P
        erroCovPost = (I - gain * j_measuremenMatrix) * erroCovPre;
        return statePost;
    }

    Eigen::Matrix<T, state_num, state_num> j_transitionMatrix;      // JA 状态转移矩阵
    Eigen::Matrix<T, measure_num, state_num> j_measuremenMatrix;    // JH 测量矩阵
    Eigen::Matrix<T, measure_num, measure_num> measurementNoiseCov; // R 测量噪声
    Eigen::Matrix<T, state_num, measure_num> gain;                  // K 卡尔曼增益
    Eigen::Matrix<T, state_num, state_num> erroCovPost;             // P'后验误差估计协方差矩阵
    Eigen::Matrix<T, state_num, state_num> erroCovPre;              // P 后验误差估计协方差矩阵
    Eigen::Matrix<T, state_num, state_num> processNoiseCov;         // Q 过程噪声
    Eigen::Matrix<T, state_num, 1> statePost;                       // X'先验状态
    Eigen::Matrix<T, state_num, 1> statePre;                        // X 后验状态
};


class EKF
{
private:
    ExtendKalmanFilter<2, 1> ekf;
    uint32_t last_timestamp;

public:
    /* data */
    struct PredictFunction
    {
        double d_time;

        template <class T>
        void operator()(T *x, T *y) //定义预测模型,y是当前(新)数据，x是上一(旧)数据
        {
            //d_time = (double)(y[1] - x[1]) / 1000;
            // x = [angle, angle_speed]
            y[0] = x[0] + 2*(x[1]-2.090+_a)
        }
        //结构体的构造函数
        PredictFunction() : d_time(0.01){}
    };
    struct MeasureFunction
    {
        template <class T>
        void operator()(T *x, T *y) //定义观测模型
        {
            // H=[1,0,0,0,0
            //    0,1,0,0,0]
            y[0] = x[0];
            y[1] = x[1];
        }
        MeasureFunction() {}
    };

    PredictFunction pre_func;
    MeasureFunction mea_func;
    EKF(const std::string &filename = "../Configure/EKF_CTRV.xml");

    /**
     *  @brief  初始化
     *  @param  world_coord 绝对系坐标
     *  @param  timestamp   时间戳
     */
    void init(cv::Point3f world_coord, uint32_t timestamp);

    /**
     *  @brief  预测更新
     *  @param  world_coord 绝对系坐标
     *  @param  timestamp   时间戳
     */
    void predict(cv::Point3f world_coord, uint32_t timestamp);

    ~EKF() {};
};

#endif