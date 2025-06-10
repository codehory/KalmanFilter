/*************************************************
* file   : kalman_filter.h
* author : ho
* date   : 25-5-3
**************************************************/


#ifndef KALMANFILTER_KALMAN_FILTER_H
#define KALMANFILTER_KALMAN_FILTER_H

#include "Eigen/Dense"
#include "data_manager.h"
#include "kalman_filter_base.h"

namespace filter {

class KF : public KalmanFilterBase {
 public:
  KF(int state_dim, int meas_dim, bool use_control = false);

  void setF(const Eigen::MatrixXd& f);
  void setH(const Eigen::MatrixXd& h);
  void setB(const Eigen::MatrixXd& b);

  void predict(const Eigen::VectorXd& u) override;
  void update(const Eigen::VectorXd& z) override;

  Eigen::MatrixXd computeKalmanGain();

 private:
  Eigen::MatrixXd F_; // state transition matrix
  Eigen::MatrixXd H_; // observation matrix
  Eigen::MatrixXd B_; //
  bool use_control_;
};


} // namespace filter


#endif //KALMANFILTER_KALMAN_FILTER_H
