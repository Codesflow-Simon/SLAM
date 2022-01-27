#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
#include <vector>
#include <map>
#include <math.h>
#include "lib/sensor.hpp"

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/MagFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

// Noise models
noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); 
noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);
noiseModel::Diagonal::shared_ptr anchor_noise_model = noiseModel::Isotropic::Sigma(3,10);
noiseModel::Diagonal::shared_ptr distance_noise_model = noiseModel::Isotropic::Sigma(1,0.1);

/* A simple function to calculate the norm of a vector 3 */
double norm(Vector3 vec) {
  double sq_x = vec.x() * vec.x();
  double sq_y = vec.y() * vec.y();
  double sq_z = vec.z() * vec.z();
  return sqrt(sq_x + sq_y + sq_z);
}

/* Transforms the json object into a map of vectors */
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

/* Creates some hyperparameters for the model */
boost::shared_ptr<PreintegratedImuMeasurements::Params> preintParams() {
  // Values were provided by examples, for IMU
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  // double accel_bias_rw_sigma = 0.004905;
  // double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8;    // error committed in integrating position from velocities

  // Define some parameters, use the matricies defined above
  auto p = PreintegratedImuMeasurements::Params::MakeSharedD(0.0);
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous

  return p;
}

/* initialises values and graph assuming tag starts at origin */
PreintegratedImuMeasurements* addIMUPriors(NonlinearFactorGraph *graph, Values *initial_values) {
  // Construct IMU priors
  Pose3 priorMean = Pose3();
  Vector3 priorVelocity = Vector3(0,0,0);
  auto priorImuBias = imuBias::ConstantBias();

  // Insert priors into values instance
  initial_values -> insert(X(0), priorMean);
  initial_values -> insert(V(0), priorVelocity);
  initial_values -> insert(B(0), priorImuBias);

  // Add priors to graph
  graph->add(PriorFactor<Pose3>(X(0), 
    priorMean, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(0),
    priorVelocity,velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(0), 
    priorImuBias,bias_noise_model));

  auto p = preintParams();
  auto preint_meas = new PreintegratedImuMeasurements(p, imuBias::ConstantBias());
  return preint_meas;
}

vector<Vector3> calibrate(int steps=100) {
  cout << "calibrating, move the tag around" << endl;
  // Prior anchor positions
  json data = getJson();
  int anchors = data["meas"]["d"].size();

  Values values;
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  auto preint_meas = addIMUPriors(graph, &values);

  auto anchorPos = vector<Vector3>(anchors);
  for (int i=0; i<anchors; i++) {
    anchorPos.at(i) = Vector3(0,0,0);
    values.insert((Key)i, Vector3(0,0,0));
    graph -> add(PriorFactor<Vector3>((Key)i, Vector3(0,0,0), anchor_noise_model));
  }
  
  NavState prev_state = NavState(Pose3(), Vector3(0,0,0));
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = imuBias::ConstantBias();

  data = getJson();
  double initial_time = data["ts"];
  double current_time = initial_time;

  auto dataList = getJsonList(steps);
  for (int index=1; index<steps; index++) {  
    if (index==steps-15) { cout << "return tag to original location" << endl; }

    data = dataList.front();
    dataList.pop_front();
    auto dataMap = dataToVecMap(data);

    double dt = (double)data["ts"] - current_time;
    current_time += dt;

    preint_meas->integrateMeasurement(dataMap["acc"], dataMap["omega"], dt);

    // Add new factors
    ImuFactor imu_factor(X(index-1), V(index-1),
                      X(index), V(index),
                      B(index-1), *preint_meas);
    graph->add(imu_factor);

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(index-1), 
                                                    B(index), 
                                                    zero_bias, bias_noise_model));

    for (int i=0; i<anchors; i++) {
      graph->add(RangeFactor<Pose3, Vector3, double>(X(index), (Key)i,
        (double)(dataMap["range"][i]), distance_noise_model));
    }

    // Add loop closure for last and first elements
    if (index == steps-1) {
      graph->add(BetweenFactor<Pose3>(X(0), X(index), Pose3(), pose_noise_model));
    }

    // Estimate next time set
    prop_state = preint_meas->predict(prev_state, prev_bias);
    
    values.insert(X(index), prop_state.pose());
    values.insert(V(index), prop_state.v());
    values.insert(B(index), prev_bias);

    // out of range error here
    LevenbergMarquardtOptimizer optimizer(*graph, values);
    Values result = optimizer.optimize();

    prev_state = NavState(result.at<Pose3>(X(index)),
                      result.at<Vector3>(V(index)));
    prev_bias = result.at<imuBias::ConstantBias>(B(index));

    preint_meas->resetIntegrationAndSetBias(prev_bias);

    for (int i=0; i<anchors; i++) {
      anchorPos.at(i) = result.at<Vector3>((Key)i);
    }
  }
  cout << "calibration done!" << endl;
  return anchorPos;
}

int main() {  
  // Load data
  json data = getJson();
  int anchors = data["meas"]["d"].size();

  int index = 0;

  auto anchorPos = calibrate();

  Values values;
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  auto preint_meas = addIMUPriors(graph, &values);

  for (int i=0; i<anchors; i++) {
    values.insert((Key)i, anchorPos.at(i));
    cout << anchorPos.at(i) << endl;
  }

  NavState prev_state = NavState(Pose3(), Vector3(0,0,0));
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = imuBias::ConstantBias();

  data = getJson();
  double initial_time = data["ts"];
  double current_time = initial_time;

  while (true) {
    index++;
    
    data = getJson();
    
    double dt = (double)data["ts"] - current_time;
    if (dt > 0.1) {
      // cout << "large dt: " << dt << endl;
    }
    current_time += dt;

    auto dataMap = dataToVecMap(data);

    preint_meas->integrateMeasurement(dataMap["acc"], dataMap["omega"], dt);

    // Add new factors
    ImuFactor imu_factor(X(index-1), V(index-1),
                      X(index), V(index),
                      B(index-1), *preint_meas);

    graph->add(imu_factor);

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(index-1), 
                                                    B(index), 
                                                    zero_bias, bias_noise_model));

    for (int i=0; i<anchors; i++) {
      graph->add(RangeFactor<Pose3, Vector3, double>(X(index), (Key)i,
        (double)(dataMap["range"][i]), distance_noise_model));

    }
    // Estimate next time set
    prop_state = preint_meas->predict(prev_state, prev_bias);
    
    values.insert(X(index), prop_state.pose());
    values.insert(V(index), prop_state.v());
    values.insert(B(index), prev_bias);

    // out of range error here
    LevenbergMarquardtOptimizer optimizer(*graph, values);
    Values result = optimizer.optimize();

    prev_state = NavState(result.at<Pose3>(X(index)),
                      result.at<Vector3>(V(index)));
    prev_bias = result.at<imuBias::ConstantBias>(B(index));

    preint_meas->resetIntegrationAndSetBias(prev_bias);

    // Print infomation
    Vector3 toTag = prev_state.t();

    cout << "i = " << index << ", dt = " << dt << endl; 
    cout << "tag at " << toTag << endl;
    // Quaternion rot = prev_state.quaternion();
    // cout << "rot " << rot.w() << ", " <<  rot.x() << ", " << rot.y() << ", " << rot.z() <<endl;

    double distanceMSE = 0;

    for (int i=0; i<anchors; i++) {
      // cout << "anchor " << i << " at " << anchorPos.at(i) << endl;
      Vector3 tagToAnchor = anchorPos.at(i) - toTag;
      double sign_err = dataMap["range"][i] - norm(tagToAnchor);
      // cout << "(norm = " << norm(tagToAnchor) << ")" << endl;
      double sq_err = sign_err * sign_err;
      distanceMSE += sq_err;
    }

    distanceMSE = distanceMSE / anchors;
      
    cout << "distance rmse: " << sqrt(distanceMSE) << endl;
  } 
  return 0;
}