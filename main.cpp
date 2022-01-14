#include <gtsam/slam/BetweenFactor.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <list>
// #include <Eigen/Dense>
#include "lib/sensor.hpp"

using namespace std;
// using namespace gtsam;

int main() {
    // Pre-measured anchor positions
    Eigen::Matrix<double,3,5> anchors;
    anchors <<  0.13,  0.65,  0.47,
                1.49, -1.06, -0.0 ,
                1.49, -0.38,  0.8 ,
               -0.17,  0.65,  0.82,
                0.31, -0.1 ,  0   ;

    return 0;
}