/*************************************************
* file   : data_manager.cpp
* author : ho
* date   : 25-5-5
**************************************************/

#include "common.h"
#include "data_manager.h"
#include <fstream>

namespace filter {
bool DataManager::loadData(const std::string& file_path, std::vector<MeasurementPack>* measurement_pack) {
  std::ifstream in_file(file_path.c_str());
  if (!in_file.is_open()) {
    std::cerr << "Cannot open file: " << file_path << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(in_file, line)) {
    std::istringstream iss(line);
    MeasurementPack meas_pack;
    std::string sensor_type;
    iss >> sensor_type;
    if (sensor_type == "L") {
      meas_pack.sensor_type = MeasurementPack::SensorType::LIDAR;
      double x;
      double y;
      iss >> x;
      iss >> y;
      meas_pack.state = Eigen::VectorXd(2);
      meas_pack.state << x, y;
    } else if (sensor_type == "R") {
      meas_pack.sensor_type = MeasurementPack::SensorType::RADAR;
      double rho;
      double phi;
      double v_rho;
      iss >> rho;
      iss >> phi;
      iss >> v_rho;
      phi = common::normalizeAngle(phi);
      meas_pack.state = Eigen::VectorXd(3);
      meas_pack.state << rho, phi, v_rho;
    }

    long long timestamp;
    iss >> timestamp;
    meas_pack.t = timestamp;

    // read ground truth
    double x_gt;
    double y_gt;
    double vx_gt;
    double vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;

    meas_pack.gt = Eigen::VectorXd(4);
    meas_pack.gt << x_gt, y_gt, vx_gt, vy_gt;
    measurement_pack->emplace_back(meas_pack);
  }

  in_file.close();
  return true;

}

bool DataManager::loadConfig(const std::string& file_path, Config* cfg) {
  YAML::Node node = YAML::LoadFile(file_path);

  cfg->F = getMatrix<Eigen::MatrixXd>(node, "F");
  cfg->P = getMatrix<Eigen::MatrixXd>(node, "P");
  cfg->H = getMatrix<Eigen::MatrixXd>(node, "H");
  cfg->Q = getMatrix<Eigen::MatrixXd>(node, "Q");
  cfg->U = getMatrix<Eigen::VectorXd>(node, "U");
  cfg->R_lidar = getMatrix<Eigen::MatrixXd>(node, "R_lidar");
  cfg->R_radar = getMatrix<Eigen::MatrixXd>(node, "R_radar");
  cfg->noise_ax = node["noise_ax"].as<double>();
  cfg->noise_ay = node["noise_ay"].as<double>();

  std::cout << cfg->F << std::endl;
  std::cout << cfg->H << std::endl;
  std::cout << cfg->R_lidar << std::endl;
  std::cout << cfg->R_radar << std::endl;
  std::cout << cfg->noise_ax << std::endl;
  std::cout << cfg->noise_ay << std::endl;


  return true;
}


}