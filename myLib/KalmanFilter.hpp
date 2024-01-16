#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class MyKalmanFilter {
public:
    MyKalmanFilter();
    ~MyKalmanFilter();

    void initialize(const Eigen::VectorXd& initial_state,
                    const Eigen::MatrixXd& initial_covariance,
                    const Eigen::MatrixXd& transition_matrix,
                    const Eigen::MatrixXd& measurement_matrix,
                    const Eigen::MatrixXd& process_noise_covariance,
                    const Eigen::MatrixXd& measurement_noise_covariance);

    void predict();
    void update(const Eigen::VectorXd& measurement);

    Eigen::VectorXd getState() const;

private:
    Eigen::VectorXd state;                   // 状态向量
    Eigen::MatrixXd covariance;              // 协方差矩阵
    Eigen::MatrixXd transition_matrix;       // 状态转移矩阵
    Eigen::MatrixXd measurement_matrix;      // 观测矩阵
    Eigen::MatrixXd process_noise_cov;      // 过程噪声协方差矩阵
    Eigen::MatrixXd measurement_noise_cov;  // 观测噪声协方差矩阵
};

#endif // KALMAN_FILTER_H
