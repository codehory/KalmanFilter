/*************************************************
* file   : ekf.cpp
* author : ho
* date   : 25-5-25
**************************************************/


#include "ekf.h"
#include "common.h"

namespace filter {
EKF::EKF(int state_dim, int meas_dim, bool use_control) : F_(Eigen::MatrixXd::Identity(state_dim, state_dim)),
                                                          H_(Eigen::MatrixXd::Zero(meas_dim, state_dim)),
                                                          B_(Eigen::MatrixXd::Zero(state_dim, state_dim)),
                                                          use_control_(use_control) {
  x_ = Eigen::VectorXd::Zero(state_dim);
  P_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  Q_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  R_ = Eigen::MatrixXd::Identity(meas_dim, meas_dim);
}

void EKF::setF(const Eigen::MatrixXd& f) {
  F_ = f;
}

void EKF::setH(const Eigen::MatrixXd& h) {
  H_ = h;
}


void EKF::predict(const Eigen::VectorXd& u) {
  if (use_control_) {
    x_ = F_ * x_ + B_ * u;
  } else {
    x_ = F_ * x_;
  }

  P_ = F_ * P_ * F_.transpose() + Q_;
}

void EKF::update(const Eigen::VectorXd& z) {
  Eigen::MatrixXd K = computeKalmanGain();
  Eigen::VectorXd x_polar(4);
  x_polar = common::cartesian2Polar(x_);
  x_ = x_ + K * (z - x_polar);
  P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}

Eigen::MatrixXd EKF::computeKalmanGain() {
  return P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
}


} // namespace filter