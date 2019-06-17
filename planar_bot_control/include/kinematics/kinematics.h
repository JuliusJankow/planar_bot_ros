#pragma once

#include <Eigen/Dense>

class Robot
{
public:
  Robot() {
    x[0] = Eigen::Vector2d::Zero();
    J[0] = Eigen::Matrix<double,2,4>::Zero();
    for(int i=0; i<4; i++) {
      link_length_.push_back(0.0);
      x[i+1] = Eigen::Vector2d::Zero();
      J[i+1] = Eigen::Matrix<double,2,4>::Zero();
    }
  }
  ~Robot(){}
  
  void setLinkLengths(const double l1, const double l2, const double l3, const double l4) {
    link_length_[0] = l1;
    link_length_[1] = l2;
    link_length_[2] = l3;
    link_length_[3] = l4;
  }
  
  void computeForwardKinematics();
  static void computeMoorePenrosePinv(const Eigen::Matrix<double,2,4>& mat, Eigen::Matrix<double,4,2>& mat_inv);

  Eigen::Vector4d q;
  Eigen::Vector4d dq;
  std::array<Eigen::Vector2d, 5> x;
  std::array<Eigen::Matrix<double,2,4> , 5> J;

private:
  std::vector<double> link_length_;

};
