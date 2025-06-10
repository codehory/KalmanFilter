/*************************************************
* file   : ekf.h
* author : ho
* date   : 25-5-25
**************************************************/


#ifndef KALMAN_FILTER_EKF_H
#define KALMAN_FILTER_EKF_H


#include "kalman_filter_base.h"

namespace filter {


class EKF : public KalmanFilterBase {
 public:
  EKF(int state_dim, int meas_dim, bool use_control = false);

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






}


#endif //KALMAN_FILTER_EKF_H
