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

} // namespace
