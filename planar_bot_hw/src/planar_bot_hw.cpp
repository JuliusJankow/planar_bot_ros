#include "planar_bot_hw.h"

using namespace planar_bot_hw;

PlanarBotHW::PlanarBotHW() {
  pos[0] = 1.7;
  pos[1] = -0.5;
  pos[2] = -0.5;
  pos[3] = 0.3;

  for(uint8_t i=0; i<kDoF; i++) {
    // connect and register the joint state interfaces
    std::string handle_name = "joint" + std::to_string(i+1);
    hardware_interface::JointStateHandle state_handle(handle_name, &pos[i], &vel[i], &eff[i]);
    jnt_state_interface.registerHandle(state_handle);
    
    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle(state_handle, &cmd[i]);
    jnt_pos_interface.registerHandle(pos_handle);
    
    cmd[i] = pos[i];
    vel[i] = 0.0;
    eff[i] = 0.0;
  }
  

  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_pos_interface);
}

void PlanarBotHW::update() {
  for(uint8_t i=0; i<kDoF; i++) {
    pos[i] = cmd[i];
  }
}
