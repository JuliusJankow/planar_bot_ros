#include "planar_bot.h"

using namespace planar_bot;

void Robot::computeFK()
{
  computeFK(q,x,J);
}

void Robot::computeFK(const Eigen::Vector4d& q_tmp, 
                      std::array< Eigen::Vector2d, 5 >& x_tmp, 
                      std::array< Eigen::Matrix<double,2,4>, 5 >& J_tmp) 
{
// efficient forward kinematics calculation for planar bot
  std::array<double,num_dof_> lc,ls;
  for(int i=0; i<num_dof_; i++) {
    double link_angle = 0.0;
    for(int j=0; j<=i; j++) {
      link_angle += q_tmp[j];
    }
    lc[i] = link_length_[i] * std::cos(link_angle);
    ls[i] = link_length_[i] * std::sin(link_angle);
  }
  
  for(int frame=1; frame<=num_dof_; frame++) { // calculate position and jacobian at [frame]
    for(int joint=0; joint<frame; joint++) { // calculate partial derivative for [joint]
      J_tmp[frame](0,joint) = 0.0;
      J_tmp[frame](1,joint) = 0.0;
      for(int i=(frame-1); i>=joint; i--) {
        J_tmp[frame](0,joint) -= ls[i];
        J_tmp[frame](1,joint) += lc[i];
      }
    }
    x_tmp[frame](0) =  J_tmp[frame](1,0);
    x_tmp[frame](1) = -J_tmp[frame](0,0);
  }
}
