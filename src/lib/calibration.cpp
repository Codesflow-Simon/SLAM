#include <nlohmann/json.hpp>
#include "src/lib/sensor.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

// Estimate prior noise to be about 20cm
noiseModel::Diagonal::shared_ptr anchor_noise_model = noiseModel::Isotropic::Sigma(3,0.2);
noiseModel::Diagonal::shared_ptr placement_noise_model = noiseModel::Isotropic::Sigma(3,0.05);
noiseModel::Diagonal::shared_ptr distance_noise_model = noiseModel::Isotropic::Sigma(1,0.1);


vector<Vector3> calibrate() {
  // Todo, save calibration

  cout << "begin calibration" << endl;
  json data = getJson();
  const int anchors = data["meas"]["d"].size();

  Values values;
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();


  values.insert((Key)0, Vector3(0,1.3,0.8));
  graph -> add(PriorFactor<Vector3>((Key)0, Vector3(0,1.3,0.8), anchor_noise_model));
  values.insert((Key)1, Vector3(1.25,2,1.5));
  graph -> add(PriorFactor<Vector3>((Key)1, Vector3(1.25,2,1.5), anchor_noise_model));
  values.insert((Key)2, Vector3(2.5,1.3,2.5));
  graph -> add(PriorFactor<Vector3>((Key)2, Vector3(2.5,1.3,2.5), anchor_noise_model));
  values.insert((Key)3, Vector3(1.5,-0.5,0));
  graph -> add(PriorFactor<Vector3>((Key)3, Vector3(1.5,-0.5,0), anchor_noise_model));
  // values.insert((Key)4, Vector3(0,-2,0));
  // graph -> add(PriorFactor<Vector3>((Key)4, Vector3(1,-2,0), anchor_noise_model));

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

    data = getJson();

    values.insert(X(i), pos);
    graph->add(PriorFactor<Pose3>(X(i), pos, placement_noise_model));  

    for (int j=0; j<anchors; j++) {
      graph->add(RangeFactor<Pose3,Vector3,double>(X(i), (Key)j , data["meas"]["d"][j], distance_noise_model));
    }
  }
  
  graph->print();

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
