#include "planar_bot_hw.h"

using namespace planar_bot_hw;

PlanarBotHW::PlanarBotHW(const std::vector<double>& initial_configuration) {
  num_dof_ = initial_configuration.size();

  cmd_ = new double[num_dof_];
  pos_ = new double[num_dof_];

  for(uint8_t i=0; i<num_dof_; i++) {
    // connect and register the joint state interfaces
    std::string handle_name = "joint" + std::to_string(i+1);
    hardware_interface::JointStateHandle state_handle(handle_name, &pos_[i], &dummy_, &dummy_);
    jnt_state_interface.registerHandle(state_handle);
    
    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle(state_handle, &cmd_[i]);
    jnt_pos_interface.registerHandle(pos_handle);
    
    pos_[i] = initial_configuration[i];
    cmd_[i] = pos_[i];
  }

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);
}

void PlanarBotHW::update() {
  for(uint8_t i=0; i<num_dof_; i++) {
    pos_[i] = cmd_[i];
  }
}

void Robot::computeForwardKinematics() {
// efficient forward kinematics calculation for planar bot
  std::vector<double> lc(num_dof_),ls(num_dof_);
  for(int i=0; i<num_dof_; i++) {
    double link_angle = 0.0;
    for(int j=0; j<=i; j++) {
      link_angle += q[j];
    }
    lc[i] = link_length_[i] * std::cos(link_angle);
    ls[i] = link_length_[i] * std::sin(link_angle);
  }
  
  for(int frame=1; frame<=num_dof_; frame++) { // calculate position and jacobian at [frame]
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
