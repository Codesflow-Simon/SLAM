#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
#include <map>
#include "lib/sensor.hpp"

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
// #include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
// #include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
// #include <gtsam/navigation/MagFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace std;
using namespace gtsam;

map<string,Vector> dataToVecMap(json jsonObj) {
  map<string, Vector> output {
    {"acc", Eigen::Matrix<double,3,1>::Zero()},
    {"omega", Eigen::Matrix<double,3,1>::Zero()},
    {"mag", Eigen::Matrix<double,3,1>::Zero()},
    {"range", Eigen::Matrix<double,5,1>::Zero()}};

  size_t accElementIndex = 0;
  for (auto& it : jsonObj["acc"]) {
    output["acc"](accElementIndex++) = it;
  }

  size_t gyroElementIndex = 0;
  for (auto& it : jsonObj["gyro"]) {
    output["omega"](gyroElementIndex++) = it;
  }

  size_t magElementIndex = 0;
  for (auto& it : jsonObj["mag"]) {
    output["mag"](magElementIndex++) = it;
  }

  size_t rangeElementIndex = 0;
  for (auto& it : jsonObj["meas"]["d"]) {
    output["range"](rangeElementIndex++) = it;
  }
  return output;
}

boost::shared_ptr<PreintegratedImuMeasurements::Params> preintParams() {
  // Values were provided by examples, for IMU
  // TODO: refine values
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  // double accel_bias_rw_sigma = 0.004905;
  // double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8;    // error committed in integrating position from velocities
  // Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  // Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  // Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5;         // error in the bias used for preintegration

  // Define some parameters, use the matricies defined above
  auto p = PreintegratedImuMeasurements::Params::MakeSharedD(0.0);
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  // p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  // p->biasAccOmegaInt = bias_acc_omega_int; ****These come with combined****

  return p;
}

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

int main() {
// Load data
  list<json> data = getJson(50);

  // Pre-measured anchor positions
  // TODO: integreate these measurements into the mode;
  Eigen::Matrix<double,3,5> anchors;
  anchors <<  0.13,  0.65,  0.47,
              1.49, -1.06, -0.0 ,
              1.49, -0.38,  0.8 ,
              -0.17,  0.65,  0.82,
              0.31, -0.1 ,  0   ;

  int index = 0;

  // Construct IMU priors
  Pose3 priorMean = Pose3();
  Vector3 priorVelocity = Vector3();
  auto priorImuBias = imuBias::ConstantBias();

  Values initial_values;
  initial_values.insert(X(index), priorMean);
  initial_values.insert(V(index), priorVelocity);
  initial_values.insert(B(index), priorImuBias);

  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); 
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);
  noiseModel::Diagonal::shared_ptr mag_noise_model = noiseModel::Isotropic::Sigma(3,1);

  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(index), 
    priorMean, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(index),
    priorVelocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(index), 
    priorImuBias,bias_noise_model));

  auto p = preintParams();

  auto preint_meas = new PreintegratedImuMeasurements(p, priorImuBias);

  NavState prev_state = NavState(priorMean, priorVelocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = priorImuBias;

  double initial_time = data.front()["ts"];
  double current_time = initial_time;
  data.pop_front();

  int size = data.size();

  for (int i=0; i<size; i++) {
    json jsonObj = data.front();
    data.pop_front();
    
    double dt = (double)jsonObj["ts"] - current_time;
    current_time += dt;

    auto dataMap = dataToVecMap(jsonObj);

    preint_meas->integrateMeasurement(dataMap["acc"], dataMap["omega"], dt);
    index++;

    ImuFactor imu_factor(X(index-1), V(index-1),
                      X(index), V(index),
                      B(index-1), *preint_meas);

    // Does nothing right now?
    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(index-1), 
                                                    B(index), 
                                                    zero_bias, bias_noise_model));

    // Estimate next time set
    prop_state = preint_meas->predict(prev_state, prev_bias);
    
    initial_values.insert(X(index), prop_state.pose());
    initial_values.insert(V(index), prop_state.v());
    initial_values.insert(B(index), prev_bias);

    LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
    Values result = optimizer.optimize();

    prev_state = NavState(result.at<Pose3>(X(index)),
                      result.at<Vector3>(V(index)));
    prev_bias = result.at<imuBias::ConstantBias>(B(index));

    preint_meas->resetIntegrationAndSetBias(prev_bias);
  }
  return 0;
}