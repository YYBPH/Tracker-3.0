#include "special.hpp"

Special::Special()
{
    // 初始化Kalman滤波器参数（根据你的需求调整）
    Eigen::VectorXd initial_state(6);  // [x, y, w, h, vx, vy]
    initial_state << 0, 0, 0, 0, 0, 0;

    // P 维度与状态向量的维度相同
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd(6, 6);
    initial_covariance << 0.1, 0, 0, 0, 0, 0,     
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;
    

    // A
    Eigen::MatrixXd transition_matrix(6, 6);
    transition_matrix << 1, 0, 0, 0, 1, 0,     
        0, 1, 0, 0, 0, 1,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    // H
    Eigen::MatrixXd measurement_matrix(4, 6);
    measurement_matrix << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0;

    // Q
    Eigen::MatrixXd process_noise_covariance(6, 6);
    process_noise_covariance << 0.1, 0, 0, 0, 0, 0,     
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;

    // R
    Eigen::MatrixXd measurement_noise_covariance(4, 4);
    measurement_noise_covariance << 0.1, 0, 0, 0,      
        0, 0.1, 0, 0,
        0, 0, 0.1, 0,
        0, 0, 0, 0.1;
    
    kalman.initialize(initial_state, initial_covariance, transition_matrix,
                      measurement_matrix, process_noise_covariance, measurement_noise_covariance);
              
    this->disap_times = 0;
}

Special::~Special() {}

void Special::ReinitKalmanFilter(cv::Rect newRect)
{
    // 初始化Kalman滤波器参数（根据你的需求调整）
    Eigen::VectorXd initial_state(6);  // [x, y, w, h, vx, vy]
    initial_state << newRect.x, newRect.y, newRect.width, newRect.height, 0, 0;
    this->newRect = newRect;            // 同时初始化，防止预测不上

    // P 维度与状态向量的维度相同
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd(6, 6);
    initial_covariance << 0.1, 0, 0, 0, 0, 0,     
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;
    

    // A
    Eigen::MatrixXd transition_matrix(6, 6);
    transition_matrix << 1, 0, 0, 0, 1, 0,     
        0, 1, 0, 0, 0, 1,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    // H
    Eigen::MatrixXd measurement_matrix(4, 6);
    measurement_matrix << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0;

    // Q
    Eigen::MatrixXd process_noise_covariance(6, 6);
    process_noise_covariance << 0.1, 0, 0, 0, 0, 0,     
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;

    // R
    Eigen::MatrixXd measurement_noise_covariance(4, 4);
    measurement_noise_covariance << 0.1, 0, 0, 0,      
        0, 0.1, 0, 0,
        0, 0, 0.1, 0,
        0, 0, 0, 0.1;
    
    kalman.initialize(initial_state, initial_covariance, transition_matrix,
                      measurement_matrix, process_noise_covariance, measurement_noise_covariance);
              
    this->disap_times = 0;

}

void Special::kalmanFilter(cv::Rect Rect)
{
    // 预测
    this->kalman.predict();

    Eigen::VectorXd measurement(4);
    measurement << Rect.x, Rect.y, Rect.width, Rect.height;

    // 校正
    this->kalman.update(measurement);

    Eigen::VectorXd predicted_state = kalman.getState();

    this->newRect = cv::Rect(predicted_state(0), predicted_state(1), predicted_state(2), predicted_state(3));

}


