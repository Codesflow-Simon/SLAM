#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
#include <vector>
#include <map>
#include <math.h>
#include "lib/sensor.hpp"
#include <unistd.h>

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

// Noise models
noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 1.0, 1.0, 1.0).finished()); // rad,rad,rad,m, m, m
noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,5); 
noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,0.1);
noiseModel::Diagonal::shared_ptr anchor_noise_model = noiseModel::Isotropic::Sigma(3,0.5);
noiseModel::Diagonal::shared_ptr distance_noise_model = noiseModel::Isotropic::Sigma(1,0.2);
noiseModel::Diagonal::shared_ptr mag_noise_model = noiseModel::Isotropic::Sigma(3,0.2);

ofstream debugLog;

/* A simple function to calculate the norm of a vector 3 */
double norm(Vector3 vec) {
  double sq_x = vec.x() * vec.x();
  double sq_y = vec.y() * vec.y();
  double sq_z = vec.z() * vec.z();
  return sqrt(sq_x + sq_y + sq_z);
}

Rot3 vectorsToRot(Unit3 a, Unit3 b) {
  Unit3 axis = a.cross(b);
  return Rot3::AlignPair(axis, a, b);
 }
/* Transforms the json object into a map of vectors */
map<string,Vector> dataToVecMap(json jsonObj) {
   const unsigned int anchors = sizeof(jsonObj["meas"]["d"]);

  map<string, Vector> output {
    {"acc", Eigen::Matrix<double,3,1>::Zero()},
    {"omega", Eigen::Matrix<double,3,1>::Zero()},
    {"mag", Eigen::Matrix<double,3,1>::Zero()},
    {"range", Eigen::Matrix<double,anchors,1>::Zero()}};
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

ISAM2 getIsam() {
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.001;
  parameters.relinearizeSkip = 5;
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

class BFieldFactor: public NoiseModelFactor1<Pose3> {
  const Rot3 measured_;
  const Rot3 initial_;

public:
  BFieldFactor(Key key, const Vector3& measured, const Rot3& initial,
    const SharedNoiseModel& model) :
    NoiseModelFactor1<Pose3>(model, key), 
    measured_(measured), initial_(initial) {}

  Vector evaluateError(const Pose3 &pose, boost::optional< Matrix & > H=boost::none) const override {
    double scale = norm(measured);
    Rot3 dir = initial * pose.rotation();
    Vector3 hq = Vector3(0,1,0) * dir * scale;


    if (H) {
      Matrix33 initial_mat = initial.matrix();
      h01 = initial_mat.coeff(0,1);
      h11 = initial_mat.coeff(1,1);
      h21 = initial_mat.coeff(2,1);

      Matrix33 pose_mat = pose.rotation().matrix();
      r00 = pose_mat.coeff(0,0);
      r10 = pose_mat.coeff(1,0);
      r20 = pose_mat.coeff(2,0);
      r01 = pose_mat.coeff(0,1);
      r11 = pose_mat.coeff(1,1);
      r21 = pose_mat.coeff(2,1);
      r02 = pose_mat.coeff(0,2);
      r12 = pose_mat.coeff(1,2);
      r22 = pose_mat.coeff(2,2);

      H* = Matrix(3,6) << 0, 0, 0, r00*h01, 0, 0, 
                          0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0
    }

    return hq - measured_;
  }
};

vector<Vector3> calibrate() {
  // Todo, save calibration
  cout << "begin calibration" << endl;
  debugLog << "fixed point calibration" << endl;

  json data = getJson();
  const int anchors = data["meas"]["d"].size();

  Values values;
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();


  values.insert((Key)0, Vector3(-2,-1,2));
  graph -> add(PriorFactor<Vector3>((Key)0, Vector3(-2,-1,2), anchor_noise_model));
  values.insert((Key)1, Vector3(-1,1.5,3));
  graph -> add(PriorFactor<Vector3>((Key)1, Vector3(-1,1.5,3), anchor_noise_model));
  values.insert((Key)2, Vector3(-2,0,1));
  graph -> add(PriorFactor<Vector3>((Key)2, Vector3(-2,0,1), anchor_noise_model));
  values.insert((Key)3, Vector3(1,-0.5,0));
  graph -> add(PriorFactor<Vector3>((Key)3, Vector3(1,-0.5,0), anchor_noise_model));
  values.insert((Key)4, Vector3(0,-2,0));
  graph -> add(PriorFactor<Vector3>((Key)4, Vector3(1,-2,0), anchor_noise_model));

  // Fixed point calibration
  for (int i=0; i<8; i++) {
    Pose3 pos;
    if (i==0) {
      cout << "place tag at origin (0,0,0)" << endl;
      pos = Pose3();
    } else if (i==1) {
      cout << "move tag to (0,1,0)" << endl;
      pos = Pose3(Rot3(), Vector3(0,1,0));
    } else if (i==2) {
      cout << "move tag to (1,1,0)" << endl;
      pos = Pose3(Rot3(), Vector3(1,1,0));
    } else if (i==3) {
      cout << "move tag to (1,0,0)" << endl;
      pos = Pose3(Rot3(), Vector3(1,0,0));
    } else if (i==4) {
      cout << "move tag to (0,0,1)" << endl;
      pos = Pose3(Rot3(), Vector3(0,0,1));
    } else if (i==5) {
      cout << "move tag to (0,1,1)" << endl;
      pos = Pose3(Rot3(), Vector3(0,1,1));
    } else if (i==6) {
      cout << "move tag to (1,1,1)" << endl;
      pos = Pose3(Rot3(), Vector3(1,1,1));
    } else if (i==7) {
      cout << "move tag to (1,0,1)" << endl;
      pos = Pose3(Rot3(), Vector3(1,0,1));
    } 
    sleep(5);
    debugLog << "calibration " << i << " " << pos << "\nmeasurements\n";

    data = getJson();
    logMeasurements(data);

    values.insert(X(i), pos);
    graph->add(PriorFactor<Pose3>(X(i), pos, pose_noise_model));  

    for (int j=0; j<anchors; j++) {
      graph->add(RangeFactor<Pose3,Vector3,double>(X(i), (Key)j , data["meas"]["d"][j], distance_noise_model));
    }
  }
  
  graph->print();
  debugLog << "before optimization" << endl;
  for (int i=0; i<anchors; i++) {
    debugLog << "anchor " << i << " at " << values.at<Vector3>((Key)i);
  }
  debugLog << "optimizing" << endl;
  LevenbergMarquardtOptimizer optimizer(*graph, values);
  values = optimizer.optimize(); 

  debugLog << "after optimization" << endl;
  vector<Vector3> anchorPos(anchors);
  for (int i=0; i<anchors; i++) {
    anchorPos.at(i) = values.at<Vector3>((Key)i);
    debugLog << "anchor " << i << " at " << values.at<Vector3>((Key)i);
  }
  cout << "calibration finishing, return to origin" << endl;
  sleep(10);
  cout << "calibration done!" << endl;
  debugLog << "calibration done" << endl;
  return anchorPos;
}

int main() {  
  debugLog.open("log", ofstream::out | ofstream::trunc);
  debugLog << "running program\n";

  // Load data
  json data = getJson();
  const int anchors = data["meas"]["d"].size();

  Rot3 initMag = vectorsToRot(Unit3(0,1,0), (Unit3)dataToVecMap(data)["mag"]);

  int index = 0;

  // auto anchorPos = calibrate();
  // Pre-calibrated values
  vector<Vector3> anchorPos(anchors);
  anchorPos.at(0) = {-2.13621, -0.479968, 1.53154};
  anchorPos.at(1) = {-0.924094, 1.3792, 2.70215};
  anchorPos.at(2) = {-1.76413, -0.168566, 1.27901};
  anchorPos.at(3) = {1.66293, -1.52572, 1.23659};
  anchorPos.at(4) = {0.266235, -1.17323, -0.774018};

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
    ImuFactor imu_factor(X(index-1), V(index-1),
                      X(index), V(index),
                      B(index-1), *preint_meas);

    graph->add(imu_factor);

    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(index-1), 
                                                    B(index), 
                                                    zero_bias, bias_noise_model));

    // Estimate next time set
    prop_state = preint_meas->predict(prev_state, prev_bias);
    
    values.insert(X(index), prop_state.pose());
    values.insert(V(index), prop_state.v());
    values.insert(B(index), prev_bias);

    debugLog << "creating B Field factor" << endl;

    graph -> add(BFieldFactor(X(index), dataMap["mag"], initMag, mag_noise_model));

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

    graph->resize(0);
    values.clear();
  } 
  return 0;
}