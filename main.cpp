#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
#include "lib/sensor.hpp"

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>


using namespace std;
using namespace gtsam;

// Shorthands, there are indexed by "index" and contain keys to relevant infomation
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

int main() {
  // Load data
  list<json> data = getJson(50);

  // Pre-measured anchor positions
  Eigen::Matrix<double,3,5> anchors;
  anchors <<  0.13,  0.65,  0.47,
              1.49, -1.06, -0.0 ,
              1.49, -0.38,  0.8 ,
              -0.17,  0.65,  0.82,
              0.31, -0.1 ,  0   ;

  // Assume intital state is at origin
  Rot3 prior_rotation = Rot3();
  Point3 prior_point = Point3();
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity = Vector3();
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

  // Insert initial values, there are assumed to be just the origin
  Values initial_values;
  int index = 0;
  initial_values.insert(X(index), prior_pose);
  initial_values.insert(V(index), prior_velocity);
  initial_values.insert(B(index), prior_imu_bias);  

  // Assemble prior noise model and add it the graph, uncertainty in initial assumptions, need correcting
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  // Add all prior factors (pose, velocity, bias) to the graph.
  // Uses X,V,B shorthands to get the keys at the index
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(index), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(index), prior_velocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(index), prior_imu_bias,bias_noise_model));

  // Need to tune these for the sensor
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  // Generate covariance matricies, identity provides uniform initially assumes no correleation?
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8;   // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5;        // error in the bias used for preintegration


  /// Set some the covariances in the model
  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  // A setting? not used anywhere in later code
  std::shared_ptr<PreintegrationType> preintegrated =
    std::make_shared<PreintegratedCombinedMeasurements>(p, prior_imu_bias);

  // Store previous state for the imu integration and the latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  NavState prop_state = prev_state; // Used to store propagated states
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // Keep track of the total error over the entire run for a simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  // Define initial time
  double initial_time = data.front()["ts"];
  double current_time = initial_time;
  data.pop_front();

  int size = data.size();

  // Loop through all measurements
  for (int i=0; i<size; i++) {
    json json_obj = data.front();
    data.pop_front();
    
    double dt = (double)json_obj["ts"] - current_time;
    current_time += dt;

    // IMU measurements
    Eigen::Matrix<double,3,1> acc = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> omega = Eigen::Matrix<double,3,1>::Zero();

    size_t accElementIndex = 0;
    for (auto& it : json_obj["acc"]) {
      acc(accElementIndex++) = it;
    }

    size_t gyroElementIndex = 0;
    for (auto& it : json_obj["gyro"]) {
      omega(gyroElementIndex++) = it;
    }

    preintegrated->integrateMeasurement(acc, omega, dt);

    // TODO: parameterise sensor number (currently fixed 5)
    // Read reange data
    Eigen::Matrix<double,5,1> range = Eigen::Matrix<double,5,1>::Zero();

    size_t rangeElementIndex = 0;
    for (auto& it : json_obj["meas"]["d"]) {
      range(rangeElementIndex++) = it;
    }

    // Creates a pointer to imu preintegrated variable
    auto preint_imu_combined = dynamic_cast<const PreintegratedCombinedMeasurements&>(*preintegrated);
    CombinedImuFactor imu_factor(X(index - 1), V(index - 1), X(index),
                                  V(index), B(index - 1), B(index),
                                  preint_imu_combined);
    graph->add(imu_factor);

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(index-1), 
                                                    B(index), 
                                                    zero_bias, bias_noise_model));

    // Add range factors here

    prop_state = preintegrated->predict(prev_state, prev_bias);

    initial_values.insert(X(index), prop_state.pose());
    initial_values.insert(V(index), prop_state.v());
    initial_values.insert(B(index), prev_bias);

    LevenbergMarquardtOptimizer optimizer(*graph, initial_values);

    Vector3 gtsam_position = prev_state.pose().translation();
    // Vector3 position_error = gtsam_position - gps.head<3>();
    // current_position_error = position_error.norm();

    // Calculate quaternion error, see example
  }

  return 0;
}