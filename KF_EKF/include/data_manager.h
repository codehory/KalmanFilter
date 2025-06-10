/*************************************************
* file   : data_manager.h
* author : ho
* date   : 25-5-5
**************************************************/


#ifndef KALMAN_FILTER_DATA_MANAGER_H
#define KALMAN_FILTER_DATA_MANAGER_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>


namespace filter {

struct Config {
  Eigen::MatrixXd F;
  Eigen::MatrixXd P;
  Eigen::MatrixXd H;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R_lidar;
  Eigen::MatrixXd R_radar;
  Eigen::VectorXd U; // control input, e.g., acceleration
  double noise_ax;
  double noise_ay;
  bool use_control;
};

struct MeasurementPack {
  enum class SensorType {
    LIDAR = 0,
    RADAR = 1,
  };

  long long t;
  SensorType sensor_type;
  Eigen::VectorXd state;  // x, y, vx, vy
  Eigen::VectorXd gt; // gt_x, gt_y, gt_vx, gt_vy
};

class DataManager {
 public:
  DataManager() = default;

  ~DataManager() = default;

  bool loadData(const std::string& file_path, std::vector<MeasurementPack>* measurement_pack);

  bool loadConfig(const std::string& file_path, Config* cfg);

  template<class T>
  T getMatrix(const YAML::Node& node, const std::string& str) {
    if (!node[str] || !node[str].IsMap()) {
      throw std::runtime_error("Missing or invalid matrix node: " + str);
    }

    auto shape = node[str]["shape"];
    auto data = node[str]["data"];

    if (!shape || !shape.IsSequence() || shape.size() != 2) {
      throw std::runtime_error("Invalid shape for matrix: " + str);
    }
    if (!data || !data.IsSequence()) {
      throw std::runtime_error("Missing or invalid data for matrix: " + str);
    }

    int rows = shape[0].as<int>();
    int cols = shape[1].as<int>();
    if (data.size() != rows * cols) {
      throw std::runtime_error("Shape does not match data size for matrix: " + str);
    }

    T mat(rows, cols);
    for (int i = 0; i < rows * cols; ++i) {
      mat(i / cols, i % cols) = data[i].as<double>();
    }
    return mat;
  }

};


} // namespace filter



#endif //KALMAN_FILTER_DATA_MANAGER_H
