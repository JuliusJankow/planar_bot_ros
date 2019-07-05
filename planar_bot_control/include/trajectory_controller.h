#pragma once

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <array>

namespace planar_bot_control
{

class SimpleController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  SimpleController();
  ~SimpleController();

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< hardware_interface::JointHandle > joints_;

private:
  Eigen::MatrixXd trajectory_;
  size_t counter_{0};

  ros::Subscriber sub_trajectory_;
  void trajectoryCB(const std_msgs::Float64MultiArrayConstPtr& msg);
}; // class

} // namespace
