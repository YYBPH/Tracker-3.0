#include "KalmanFilter.hpp"

MyKalmanFilter::MyKalmanFilter() {}
MyKalmanFilter::~MyKalmanFilter() {}

void MyKalmanFilter::initialize(const Eigen::VectorXd& initial_state,
                               const Eigen::MatrixXd& initial_covariance,
                               const Eigen::MatrixXd& transition_matrix,
                               const Eigen::MatrixXd& measurement_matrix,
                               const Eigen::MatrixXd& process_noise_covariance,
                               const Eigen::MatrixXd& measurement_noise_covariance) {
    this->state = initial_state;
    this->covariance = initial_covariance;
    this->transition_matrix = transition_matrix;
    this->measurement_matrix = measurement_matrix;
    this->process_noise_cov = process_noise_covariance;
    this->measurement_noise_cov = measurement_noise_covariance;
}

void MyKalmanFilter::predict() {
    // 预测步骤
    state = transition_matrix * state;
    covariance = transition_matrix * covariance * transition_matrix.transpose() + process_noise_cov;
}

void MyKalmanFilter::update(const Eigen::VectorXd& measurement) {
    // 更新步骤
    Eigen::MatrixXd kalman_gain = covariance * measurement_matrix.transpose() *
                                  (measurement_matrix * covariance * measurement_matrix.transpose() +
                                   measurement_noise_cov).inverse();

    state = state + kalman_gain * (measurement - measurement_matrix * state);
    covariance = (Eigen::MatrixXd::Identity(state.size(), state.size()) - kalman_gain * measurement_matrix) * covariance;
}

Eigen::VectorXd MyKalmanFilter::getState() const {
    return state;
}

