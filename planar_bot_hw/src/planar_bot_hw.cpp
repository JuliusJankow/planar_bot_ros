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
