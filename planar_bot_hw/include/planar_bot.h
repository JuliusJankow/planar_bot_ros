#pragma once

#include <array>
#include <Eigen/Dense>

namespace planar_bot
{

struct Robot
{
  Robot() : num_dof_(4) {
    for(int i=0; i<num_dof_+1; i++) {
      x[i] = Eigen::Vector2d::Zero();
      J[i] = Eigen::Matrix<double,2,4>::Zero();
    }
  }
  
  void setLinkGeometry(const std::array<double,4>& link_length, 
                       const std::array<double,4>& link_radius) {
    link_length_ = link_length; link_radius_ = link_radius; }
                       
  void setJointLimits(const std::array<double,4>& q_min,
                      const std::array<double,4>& q_max,
                      const std::array<double,4>& dq_max,
                      const std::array<double,4>& ddq_max) {
    q_min_ = q_min; q_max_ = q_max; 
    dq_max_ = dq_max; ddq_max_ = ddq_max; }

  void computeFK();
  void computeFK(const Eigen::Vector4d& q_tmp, 
                 std::array< Eigen::Vector2d, 5 >& x_tmp, 
                 std::array< Eigen::Matrix<double,2,4>, 5 >& J_tmp);

  Eigen::Vector4d q;
  std::array< Eigen::Vector2d, 5 > x;
  std::array< Eigen::Matrix<double,2,4>, 5 > J;

  int num_dof_;
  
  std::array<double,4> link_length_, link_radius_; // for ssv usage
  std::array<double,4> q_min_, q_max_, dq_max_, ddq_max_; // joint limits

}; // class

} // namespace
