/*************************************************
* file   : common.h
* author : ho
* date   : 25-5-26
**************************************************/


#ifndef KALMAN_FILTER_COMMON_H
#define KALMAN_FILTER_COMMON_H

#include <Eigen/Dense>

namespace common {

inline double normalizeAngle(double phi) {
  if (phi > M_PI) {
    phi -= 2 * M_PI;
  } else if (phi < -M_PI) {
    phi += 2 * M_PI;
  }

  return phi;
}

inline Eigen::VectorXd cartesian2Polar(const Eigen::VectorXd& x) {

  Eigen::VectorXd x_polar(3);

  double px = x[0];
  double py = x[1];
  double rho = std::sqrt(px * px + py * py);
  double phi = std::atan2(py, px);

  if (rho < 1e-6) rho = 1e-6;

  // Calculate the radial velocity:  (px, py) dot. (vx, vy) / rho
  double v_rho = (px * x[2] + py * x[3]) / rho;

  x_polar << rho, phi, v_rho;

  return x_polar;
}

inline Eigen::VectorXd polar2Cartesian(const Eigen::VectorXd& x) {

  Eigen::VectorXd x_cartesian(4);

  double c = std::cos(x[1]);
  double s = std::sin(x[1]);
  double px = x[0] * c;
  double py = x[0] * s;
  double vx = x[2] * c;
  double vy = x[2] * s;

  x_cartesian << px, py, vx, vy;

  return x_cartesian;
}


}


#endif //KALMAN_FILTER_COMMON_H
