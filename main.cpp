#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
#include <map>
#include "lib/sensor.hpp"

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/MagFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>


using namespace std;
using namespace gtsam;

// Keys: https://gtsam.org/doxygen/a03436.html)
using symbol_shorthand::X; // These given an index cast to a key (large number, assumed to be unquie)
using symbol_shorthand::V; // This allows us to use one time series index to track several catagories of values
using symbol_shorthand::B; 
using symbol_shorthand::M; 

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

int main() {
  // Load data
  list<json> data = getJson(50);
  auto initialData = dataToVecMap(data.front());

  // Pre-measured anchor positions
  // TODO: integreate these measurements into the mode;
  Eigen::Matrix<double,3,5> anchors;
  anchors <<  0.13,  0.65,  0.47,
              1.49, -1.06, -0.0 ,
              1.49, -0.38,  0.8 ,
              -0.17,  0.65,  0.82,
              0.31, -0.1 ,  0   ;

  // Tune to location (http://www.ngdc.noaa.gov/geomag-web/#igrfwmm)
  // Measured north, east, up
  Unit3 localBField {17879.4,	4907.8,	-59153.3};

  // Assume sensor starts at origin
  Pose3 prior_pose =  Pose3();
  Vector3 prior_velocity = Vector3({0,0,0});
  imuBias::ConstantBias prior_imu_bias; // Default constuctor means zero bias
  Vector3 prior_mag = initialData["mag"];

  // Values store some value at an index, we use keys (though symbol shorthands) to access these value
  Values initial_values;
  int index = 0;
  initial_values.insert(X(index), prior_pose);
  initial_values.insert(V(index), prior_velocity);
  initial_values.insert(B(index), prior_imu_bias);  
  initial_values.insert(M(index), prior_mag);  

  // These were the noise specs used in examples
  // TODO: refine noise specs
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); 
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);
  noiseModel::Diagonal::shared_ptr mag_noise_model = noiseModel::Isotropic::Sigma(3,5);

  // Add priors to graph
  // (-> is like . operator but for pointers)
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(index), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(index), prior_velocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(index), prior_imu_bias,bias_noise_model));
  graph->add(PriorFactor<Vector3>(M(index), prior_mag, mag_noise_model));
  
  auto p = preintParams();

  // This keeps and manages our preintegrated data
  auto preint_meas = new PreintegratedImuMeasurements(p, prior_imu_bias);

  // Store previous state for the imu integration and the latest predicted outcome.
  NavState prev_state = NavState(prior_pose, prior_velocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // Define initial time
  double initial_time = data.front()["ts"];
  double current_time = initial_time;
  data.pop_front();

  int size = data.size();

  // Loop through all measurements
  for (int i=0; i<size; i++) {
    json jsonObj = data.front();
    data.pop_front();
    
    double dt = (double)jsonObj["ts"] - current_time;
    current_time += dt;

    auto dataMap = dataToVecMap(jsonObj);

    // Add IMU factors to graph
    preint_meas->integrateMeasurement(dataMap["acc"], dataMap["omega"], dt);
    index++;

    PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(preint_meas);
    ImuFactor imu_factor(X(index-1), V(index-1),
                         X(index), V(index),
                         B(index-1), *preint_imu);
    graph->add(imu_factor);

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(index-1), 
                                                    B(index), 
                                                    zero_bias, bias_noise_model));
    
    // Add mag factors to graph
    MagFactor mag_factor(M(index), dataMap["mag"])
                                                    

    // Add range factors here

    prop_state = preint_meas->predict(prev_state, prev_bias);

    initial_values.insert(X(index), prop_state.pose());
    initial_values.insert(V(index), prop_state.v());
    initial_values.insert(B(index), prev_bias);

    // cout << "before optimization: "<< prev_state << endl;

    LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
    Values result = optimizer.optimize();

    // Overwrite the beginning of the preintegration for the next step.
    prev_state = NavState(result.at<Pose3>(X(index)),
                          result.at<Vector3>(V(index)));
    prev_bias = result.at<imuBias::ConstantBias>(B(index));

    // Reset the preintegration object.
    preint_meas->resetIntegrationAndSetBias(prev_bias);

    cout << prev_state << endl;
    // Vector3 position_error = gtsam_position - gps.head<3>();
    // current_position_error = position_error.norm();

    // Calculate quaternion error, see example
  }

  return 0;
}