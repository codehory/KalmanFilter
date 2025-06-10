/*************************************************
* file   : main.cpp
* author : ho
* date   : 25-5-3
**************************************************/

#include <iostream>
#include <fstream>
#include "kf.h"
#include "ekf.h"
#include "data_manager.h"
#include "common.h"

using namespace filter;


Eigen::VectorXd cvtState(const Eigen::VectorXd& x) {
  return (x.size() == 3) ? common::polar2Cartesian(x) : x;
}

void dumpState(const Eigen::VectorXd& x, const Eigen::VectorXd& gt, const long long t, std::ofstream* fo) {
  if (x.size() != 4) {
    throw std::invalid_argument("Input vector must have exactly 4 elements.");
  }

  *fo << t << " " << x.transpose() << " " << gt.transpose() << std::endl;
}


Eigen::MatrixXd calculateJacobian(const Eigen::VectorXd& x) {
  if (x.size() != 3 && x.size() != 4) {
    throw std::invalid_argument("Input vector must have exactly 3 or 4 elements.");
  }

  Eigen::VectorXd x_cartesian = cvtState(x);

  double px = x_cartesian(0);
  double py = x_cartesian(1);
  double vx = x_cartesian(2);
  double vy = x_cartesian(3);
  double c1 = px * px + py * py;
  if (c1 < 1e-12) c1 = 1e-12;
  double c2 = std::sqrt(c1);
  double c3 = c1 * c2;

  Eigen::MatrixXd h_j(3, 4);

  h_j << px / c2, py / c2, 0, 0,
      -py / c1, px / c1, 0, 0,
      py * (py * vx - px * vy) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return h_j;
}


int main(int argc, char** argv) {
  std::ofstream fo("../data/output.txt");
  if (!fo.is_open()) {
    std::cerr << "Failed to open output file!" << std::endl;
    return -1;
  }

  const std::string data_path = "../data/sample-laser-radar-measurement-data-1.txt";
  const std::string cfg_path = "../config/cfg.yaml";

  DataManager data_manager;
  std::vector<MeasurementPack> measurement_pack;
  data_manager.loadData(data_path, &measurement_pack);
  if (measurement_pack.empty()) return -1;

  Config cfg;
  data_manager.loadConfig(cfg_path, &cfg);

  KF kf(4, 4, false);
  kf.setH(cfg.H);
  kf.setP(cfg.P);
  kf.setQ(cfg.Q);
  kf.setR(cfg.R_lidar);

  EKF ekf(4, 3, false);

  bool is_initialized = false;
  long long last_time{0};
  Eigen::VectorXd last_state(4);

  for (const auto& meas_pack : measurement_pack) {
    if (!is_initialized) {
      if (meas_pack.sensor_type == MeasurementPack::SensorType::LIDAR) {
        last_state << meas_pack.state(0), meas_pack.state(1), 0, 0;
        kf.setState(last_state);
      } else if (meas_pack.sensor_type == MeasurementPack::SensorType::RADAR) {
        last_state = cvtState(meas_pack.state);
        ekf.setState(last_state);
      } else {
        std::cerr << "Unknown sensor_type: " << std::endl;
        continue;
      }

      is_initialized = true;
      last_time = meas_pack.t;
      dumpState(last_state, meas_pack.gt, meas_pack.t, &fo);
      continue;
    }

    double dt = static_cast<double>(meas_pack.t - last_time) / 1000000.0;

    Eigen::MatrixXd F = cfg.F;
    F(0, 2) = dt;
    F(1, 3) = dt;

    const double dt_2 = dt * dt;
    const double dt_3 = dt_2 * dt;
    const double dt_4 = dt_3 * dt;
    Eigen::MatrixXd Q(4, 4);
    Q << 0.25 * dt_4 * cfg.noise_ax, 0, 0.5 * dt_3 * cfg.noise_ax, 0,
        0, 0.25 * dt_4 * cfg.noise_ay, 0, 0.5 * dt_3 * cfg.noise_ay,
        0.5 * dt_3 * cfg.noise_ax, 0, dt_2 * cfg.noise_ax, 0,
        0, 0.5 * dt_3 * cfg.noise_ay, 0, dt_2 * cfg.noise_ay;

    if (meas_pack.sensor_type == MeasurementPack::SensorType::LIDAR) {
      Eigen::VectorXd x(4);
      x = cvtState(last_state);
      kf.setF(F);
      kf.setQ(Q);
      kf.setR(cfg.R_lidar);
      kf.setState(x);
      kf.predict(cfg.U);
      kf.update(meas_pack.state);
      last_state = kf.getState();
      dumpState(last_state, meas_pack.gt, meas_pack.t, &fo);
    } else if (meas_pack.sensor_type == MeasurementPack::SensorType::RADAR) {
      Eigen::VectorXd x = cvtState(last_state);
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 4);
      H = calculateJacobian(x);
      ekf.setF(F);
      ekf.setH(H);
      ekf.setQ(Q);
      ekf.setR(cfg.R_radar);
      ekf.setState(x);
      ekf.predict(cfg.U);
      ekf.update(meas_pack.state);
      last_state = cvtState(ekf.getState());
      dumpState(last_state, meas_pack.gt, meas_pack.t, &fo);
    } else {
      std::cerr << "Unknown sensor_type: " << std::endl;
      continue;
    }

    last_time = meas_pack.t;
  }

  fo.close();
  return 0;
}