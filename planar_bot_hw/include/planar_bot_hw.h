#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace planar_bot_hw
{

static constexpr uint8_t kDoF = 4;

class PlanarBotHW : public hardware_interface::RobotHW {
public:
  PlanarBotHW();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  
  double cmd[kDoF];
  double pos[kDoF];
  double vel[kDoF];
  double eff[kDoF];
  /*std::array<double, kDoF> pos;
  double dummy{0.0};*/
}; // class

} // namespace
