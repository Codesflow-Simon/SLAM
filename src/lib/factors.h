#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
;
using namespace gtsam;
using namespace std;

class BFieldFactor: public NoiseModelFactor1<Pose3> {
  const Vector3 measured_;
  const Rot3 initial_;

public:
  BFieldFactor(Key key, const Vector3& measured, const Rot3& initial,
    const SharedNoiseModel& model) :
    NoiseModelFactor1<Pose3>(model, key), 
    measured_(measured), initial_(initial) {}

  Vector evaluateError(const Pose3 &pose, boost::optional< Matrix & > H=boost::none) const override {
    double scale = norm(measured_);
    Rot3 dir = initial_ * pose.rotation();
    Vector3 hq = scale * dir.matrix() * Vector3(0,1,0);

    if (H) {
      Matrix33 initial_mat = initial_.matrix();
      double h01 = initial_mat.coeff(0,1);
      double h11 = initial_mat.coeff(1,1);
      double h21 = initial_mat.coeff(2,1);

      Matrix33 pose_mat = pose.rotation().matrix();
      double r00 = pose_mat.coeff(0,0);
      double r10 = pose_mat.coeff(1,0);
      double r20 = pose_mat.coeff(2,0);
      double r01 = pose_mat.coeff(0,1);
      double r11 = pose_mat.coeff(1,1);
      double r21 = pose_mat.coeff(2,1);
      double r02 = pose_mat.coeff(0,2);
      double r12 = pose_mat.coeff(1,2);
      double r22 = pose_mat.coeff(2,2);

      *H = (gtsam::Matrix(3,6) << 0, 0, 0, r00*h01, r01*h11, r02*h21, 
                                  0, 0, 0, r10*h01, r11*h11, r12*h21,
                                  0, 0, 0, r20*h01, r21*h11, r22*h21).finished();
    }

    return hq - measured_;
  }
};