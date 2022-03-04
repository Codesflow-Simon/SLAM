#include <iostream>
#include <nlohmann/json.hpp>
#include <vector>
#include <map>
#include <math.h>
#include <unistd.h>

#include "src/lib/sensor.h"
#include "src/lib/util.h"
// #include "src/lib/calibration.h"
#include "src/lib/factors.h"

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

// Noise models
noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 1.0, 1.0, 1.0).finished()); // rad,rad,rad,m, m, m
noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.05); 
noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,0.1);
noiseModel::Diagonal::shared_ptr mag_noise_model = noiseModel::Isotropic::Sigma(3, 0.2);
noiseModel::Diagonal::shared_ptr distance_noise_model = noiseModel::Isotropic::Sigma(1, 0.1);
noiseModel::Diagonal::shared_ptr anchor_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);

ofstream debugLog;

Rot3 vectorsToRot(Unit3 a, Unit3 b) {
  Unit3 axis = a.cross(b);
  return Rot3::AlignPair(axis, a, b);
 }
/* Transforms the json object into a map of vectors */
/* Creates some hyperparameters for the model */
boost::shared_ptr<PreintegratedCombinedMeasurements::Params> preintParams() {
  // Values were provided by examples, for IMU
  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.1;
  double gyro_noise_sigma = 0.1;
  double accel_bias_rw_sigma = 0.1;
  double gyro_bias_rw_sigma = 0.1;
  Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
  Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
  Matrix33 integration_error_cov = Matrix33::Identity(3,3)*0.01; // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
  Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
  Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*0.01; // error in the bias used for preintegration

  // Define some parameters, use the matricies defined above
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  return p;
}

/* initialises values and graph assuming tag starts at origin */
PreintegratedCombinedMeasurements* addIMUPriors(NonlinearFactorGraph *graph, Values *initial_values) {
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
  auto preint_meas = new PreintegratedCombinedMeasurements(p, priorImuBias);
  return preint_meas;
}

ISAM2 getIsam() {
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.001;
  parameters.relinearizeSkip = 1;
  parameters.cacheLinearizedFactors = false;
  parameters.enableDetailedResults = true;
  ISAM2 isam(parameters);
  return isam;
}

void logMeasurements(json data) {
  debugLog << "acc = " << data["acc"] << endl;
  debugLog << "omega = " << data["gyro"] << endl;
  debugLog << "range = " << data["meas"]["d"] << endl;
  debugLog << "mag = " << data["mag"] << endl;
}

int main() {  
  debugLog.open("log", ofstream::out | ofstream::trunc);
  debugLog << "running program" << endl;


  // Load data
  json data = getJson();
  const int anchors = data["meas"]["d"].size();

  Rot3 initMag = vectorsToRot(Unit3(0,1,0), (Unit3)dataToVecMap(data)["mag"]);

  int index = 0;

  debugLog << "locating " << anchors << " anchors" << endl;
  // auto anchorPos = calibrate();
  // // Pre-calibrated values
  vector<Vector3> anchorPos(anchors);
  anchorPos.at(0) = {-0.313173, -0.148878, 1.9611};
  anchorPos.at(1) = {-1.07305, 0.973571, 2.21429};
  anchorPos.at(2) = {-2.53415, 0.57387, 0.459788};
  anchorPos.at(3) = {0.605421, -0.342556, -0.358743};
  // anchorPos.at(4) = {0.266235, -1.17323, -0.774018};

  debugLog << "creating structures" << endl;

  Values values;
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  ISAM2 isam = getIsam();
  auto preint_meas = addIMUPriors(graph, &values);

  debugLog << "creating anchor priors" << endl;

  for (int i=0; i<anchors; i++) {
    values.insert((Key)i, anchorPos.at(i));
    graph -> add(PriorFactor<Vector3>((Key)i, anchorPos.at(i), anchor_noise_model));
  }

  NavState prev_state = NavState(Pose3(), Vector3(0,0,0));
  NavState prop_state = prev_state;
  NavState del_state = prev_state;
  imuBias::ConstantBias prev_bias = imuBias::ConstantBias();

  data = getJson();
  double initial_time = data["ts"];
  double current_time = initial_time;

  while (true) {
    index++;
    debugLog << "loop = " << index << endl;
    
    debugLog << "getting data" << endl;
    data = getJson();
    logMeasurements(data);
    
    double dt = (double)data["ts"] - current_time;
    if (dt > 0.1) {
      // cout << "large dt: " << dt << endl;
    }
    current_time += dt;

    auto dataMap = dataToVecMap(data);

    debugLog << "creating IMU factor" << endl;

    preint_meas->integrateMeasurement(dataMap["acc"], dataMap["omega"], dt);

    // Add new factors
    CombinedImuFactor imu_factor(X(index-1), V(index-1),
                      X(index), V(index),
                      B(index-1), B(index),
                       *preint_meas);

    // graph->add(imu_factor);

    // Estimate next time set
    // Pass initial state
    prop_state = preint_meas->predict(prev_state, prev_bias);

    debugLog << "prev state: " << prev_state << endl;
    debugLog << "prop state: " << prop_state << endl;
    
    values.insert(X(index), prop_state.pose());
    // values.insert(V(index), prop_state.v());
    // values.insert(B(index), prev_bias);

    // debugLog << "creating B Field factor" << endl;

    // graph -> add(BFieldFactor(X(index), dataMap["mag"], initMag, mag_noise_model));

    debugLog << "creating range factors" << endl;

    // Sensors can miss a measurement
    if (data["meas"]["d"].size() != (unsigned int)anchors) { continue; }

    for (int i=0; i<anchors; i++) {
      graph->add(RangeFactor<Pose3, Vector3, double>(X(index), (Key)i,
        dataMap["range"][i], distance_noise_model));
    }

    debugLog << "before optimisation" << endl;
    debugLog << "pose: " << prop_state.pose() << endl;
    debugLog << "vel: " << prop_state.v() << endl;
    debugLog << "bias: " << prev_bias << endl;

    // out of range error here
    ISAM2Result result = isam.update(*graph, values);
    result.print();

    NonlinearFactorGraph temp_graph = isam.getFactorsUnsafe();
    Values best = isam.calculateBestEstimate();
    
    debugLog << "pose: " << best.at<Vector3>((Key)0) << endl;

    graph->resize(0);
    values.clear();
  } 
  return 0;
}