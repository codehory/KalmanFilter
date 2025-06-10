/*************************************************
* file   : kalman_filter.cpp
* author : ho
* date   : 25-5-3
**************************************************/


#include "kf.h"

namespace filter {

KF::KF(int state_dim, int meas_dim, bool use_control) : F_(Eigen::MatrixXd::Identity(state_dim, state_dim)),
                                                        H_(Eigen::MatrixXd::Zero(meas_dim, state_dim)),
                                                        B_(Eigen::MatrixXd::Zero(state_dim, state_dim)),
                                                        use_control_(use_control) {
  x_ = Eigen::VectorXd::Zero(state_dim);
  P_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  Q_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  R_ = Eigen::MatrixXd::Identity(meas_dim, meas_dim);
}

void KF::setF(const Eigen::MatrixXd& f) {
  F_ = f;
}

void KF::setH(const Eigen::MatrixXd& h) {
  H_ = h;
}

void KF::setB(const Eigen::MatrixXd& b) {
  B_ = b;
  use_control_ = true;
}

void KF::predict(const Eigen::VectorXd& u) {
  if (use_control_) {
    x_ = F_ * x_ + B_ * u;
  } else {
    x_ = F_ * x_;
  }

  P_ = F_ * P_ * F_.transpose() + Q_;
}

Eigen::MatrixXd KF::computeKalmanGain() {
  return P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
}


void KF::update(const Eigen::VectorXd& z) {
  Eigen::MatrixXd K = computeKalmanGain();
  x_ = x_ + K * (z - H_ * x_);
  P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}

} // namespace filter