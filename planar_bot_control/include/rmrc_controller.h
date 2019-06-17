#ifndef RMRC_CONTROLLER_H
#define RMRC_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <array>
#include <planning/spline.h>
#include <kinematics/kinematics.h>

namespace planar_bot_control
{

class RMRCController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  RMRCController();
  ~RMRCController();

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< hardware_interface::JointHandle > joints_;

private:
  Robot robot_;

  ros::Subscriber sub_task_space_goal_;

  Eigen::Matrix2d K_e_;

  Spline spline_;

  void goalCB(const std_msgs::Float64MultiArrayConstPtr& msg);
}; // class

} // namespace

#endif
