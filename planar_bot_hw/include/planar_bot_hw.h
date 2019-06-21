#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <vector>
#include <Eigen/Dense>

namespace planar_bot_hw
{

class PlanarBotHW : public hardware_interface::RobotHW {
public:
  PlanarBotHW(const std::vector<double>& initial_configuration);
  ~PlanarBotHW() {
    delete cmd_;
    delete pos_;  }
  void update();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  
  int num_dof_{0};
  
  double* cmd_;
  double* pos_;
  double dummy_{0.0};
}; // class

// TODO: Make a custom interface containing x and J
class Robot
{
public:
  Robot(const std::vector<double>& link_length) : link_length_(link_length) {
    num_dof_ = link_length.size();
    for(int i=0; i<num_dof_+1; i++) {
      x.emplace_back(Eigen::Vector2d::Zero());
      J.emplace_back(Eigen::Matrix<double,2,4>::Zero());
    }
  }
  ~Robot(){}
  
  void computeForwardKinematics();
  static void computeMoorePenrosePinv(const Eigen::Matrix<double,2,4>& mat, Eigen::Matrix<double,4,2>& mat_inv);

  Eigen::Vector4d q;
  std::vector<Eigen::Vector2d> x;
  std::vector<Eigen::Matrix<double,2,4> > J;

private:
  int num_dof_;
  std::vector<double> link_length_;

}; // class

} // namespace
