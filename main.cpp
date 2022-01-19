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

  PreintegrationType *imu_preintegrated_;

  // Assume intital state is at origin
  Rot3 prior_rotation = Rot3();
  Point3 prior_point = Point3();
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity = Vector3();
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_velocity);
  initial_values.insert(B(correction_count), prior_imu_bias);  

  // Assemble prior noise model and add it the graph.
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

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

  imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);

    // Store previous state for the imu integration and the latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // Keep track of the total error over the entire run for a simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  // Error catch for type errors here
  double initial_time = data.front()["ts"];
  double current_time = initial_time;
  data.pop_front();

  int size = data.size();

  for (int i=0; i<size; i++) {
    json json_obj = data.front();
    data.pop_front();
    
    double dt = (double)json_obj["ts"] - current_time;

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

    imu_preintegrated_->integrateMeasurement(acc, omega, dt);

    // TODO: parameterise sensor number (currently fixed 5)
    // Read reange data
    Eigen::Matrix<double,5,1> range = Eigen::Matrix<double,5,1>::Zero();

    size_t rangeElementIndex = 0;
    for (auto& it : json_obj["meas"]["d"]) {
      range(rangeElementIndex++) = it;
    }

    PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
    ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                          X(correction_count  ), V(correction_count ),
                          B(correction_count-1),
                          *preint_imu);
    graph->add(imu_factor);

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1), 
                                                    B(correction_count  ), 
                                                    zero_bias, bias_noise_model));

    // Add range factors here

    prop_state = imu_preintegrated_->predict(prev_state, prev_bias);

    initial_values.insert(X(correction_count), prop_state.pose());
    initial_values.insert(V(correction_count), prop_state.v());
    initial_values.insert(B(correction_count), prev_bias);

    LevenbergMarquardtOptimizer optimizer(*graph, initial_values);

    Vector3 gtsam_position = prev_state.pose().translation();
    Vector3 position_error = gtsam_position - gps.head<3>();
    current_position_error = position_error.norm();

    // Calculate quaternion error, see example
  }

  return 0;
}