/*************************************************
* file   : kalman_filter_base.h
* author : ho
* date   : 25-5-24
**************************************************/


#ifndef KALMAN_FILTER_KALMAN_FILTER_BASE_H
#define KALMAN_FILTER_KALMAN_FILTER_BASE_H

#include <Eigen/Dense>

namespace filter {

class KalmanFilterBase {
 public:
  virtual ~KalmanFilterBase() = default;

  virtual void predict(const Eigen::VectorXd& u) = 0;
  virtual void update(const Eigen::VectorXd& z) = 0;

  void setState(const Eigen::VectorXd& x) { x_ = x; }
  void setP(const Eigen::MatrixXd& p) { P_ = p; }
  void setQ(const Eigen::MatrixXd& q) { Q_ = q; }
  void setR(const Eigen::MatrixXd& r) { R_ = r; }

  Eigen::VectorXd getState() const { return x_; }
  Eigen::MatrixXd getCovarianceMatrix() const { return P_; }

 protected:
  Eigen::VectorXd x_; // state vector
  Eigen::MatrixXd P_; // state covariance matrix
  Eigen::MatrixXd Q_; // process noise covariance matrix
  Eigen::MatrixXd R_; // measurement noise covariance matrix
};


} // namespace filter


#endif //KALMAN_FILTER_KALMAN_FILTER_BASE_H
