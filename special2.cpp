#include "special2.hpp"

Special2::Special2()
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
    Eigen::MatrixXd measurement_matrix(6, 6);
    measurement_matrix << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    // Q
    Eigen::MatrixXd process_noise_covariance(6, 6);
    process_noise_covariance << 0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;

    // R
    Eigen::MatrixXd measurement_noise_covariance(6, 6);
    process_noise_covariance << 0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;

    kalman.initialize(initial_state, initial_covariance, transition_matrix,
        measurement_matrix, process_noise_covariance, measurement_noise_covariance);

    this->disap_times = 0;
    this->lastX = 0;
    this->lastY = 0;
}

Special2::~Special2() {}

void Special2::ReinitKalmanFilter(cv::Rect newRect)
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
    Eigen::MatrixXd measurement_matrix(6, 6);
    measurement_matrix << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    // Q
    Eigen::MatrixXd process_noise_covariance(6, 6);
    process_noise_covariance << 0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;

    // R
    Eigen::MatrixXd measurement_noise_covariance(6, 6);
    process_noise_covariance << 0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;

    kalman.initialize(initial_state, initial_covariance, transition_matrix,
        measurement_matrix, process_noise_covariance, measurement_noise_covariance);

    this->disap_times = 0;
    this->lastX = 0;
    this->lastY = 0;

}

void Special2::kalmanFilter(cv::Rect Rect)
{
    // 预测
    this->kalman.predict();

    Eigen::VectorXd measurement(6);
    measurement << Rect.x, Rect.y, Rect.width, Rect.height, lastX-Rect.x, lastY-Rect.y;

    // 校正
    this->kalman.update(measurement);

    Eigen::VectorXd predicted_state = kalman.getState();

    // 观测矩阵究竟是什么？
    this->newRect = cv::Rect(predicted_state(0), predicted_state(1), predicted_state(2), predicted_state(3));

}


