#include <iostream>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <random>

#include "util.h"

using namespace std;
using namespace Eigen;

class Emulator {
    private:
        list<Vector3> anchors;
        double error;
        default_random_engine e (0); 
        normal_distribution<double> distN(0,1); 

    public:
        void setAnchor(Vector3 t) {
            anchors.push_front(t);    
        }
        void setMeasurementError(double sigma) {
            error = sigma;
        }
        
        vector<double> sample(Vector3 tag) {
            vector<double> output = vector<double>(anchors.size());
            for (int i=0; i<anchors.size(); i++) {
                double noise = distN(e);
            }

        }
}