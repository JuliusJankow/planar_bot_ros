#include "kinematics/kinematics.h"

void Robot::computeForwardKinematics() {
// efficient forward kinematics calculation for planar bot
  std::vector<double> lc,ls;
  for(int i=0; i<4; i++) {
    double link_angle = 0.0;
    for(int j=0; j<=i; j++) {
      link_angle += q[j];
    }
    lc.emplace_back(link_length_[i] * std::cos(link_angle));
    ls.emplace_back(link_length_[i] * std::sin(link_angle));
  }
  
  for(int frame=1; frame<=4; frame++) { // calculate position and jacobian at [frame]
    for(int joint=0; joint<frame; joint++) { // calculate partial derivative for [joint]
      J[frame](0,joint) = 0.0;
      J[frame](1,joint) = 0.0;
      for(int i=(frame-1); i>=joint; i--) {
        J[frame](0,joint) -= ls[i];
        J[frame](1,joint) += lc[i];
      }
    }
    x[frame](0) =  J[frame](1,0);
    x[frame](1) = -J[frame](0,0);
  }
}

void Robot::computeMoorePenrosePinv(const Eigen::Matrix<double,2,4>& mat, Eigen::Matrix<double,4,2>& mat_inv) {
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(mat);
  mat_inv = cod.pseudoInverse();
}
