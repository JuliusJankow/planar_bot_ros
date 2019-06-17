#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>
#include <Eigen/Dense>
#include <Eigen/QR>

#include "rmrc_controller.h"

namespace planar_bot_control {

RMRCController::RMRCController() {}
RMRCController::~RMRCController() {sub_task_space_goal_.shutdown();}

bool RMRCController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
  std::vector< std::string > joint_names;
  // List of controlled joints
  std::string param_name = "joints";
  if(!n.getParam(param_name, joint_names)) {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }

  if(joint_names.size() == 0) {
    ROS_ERROR_STREAM("List of joint names is empty.");
    return false;
  }
  
  double k_e = 0.0;
  if(!n.getParam("drift_compensation", k_e)) {
    ROS_WARN_STREAM("Failed to getParam drift_compensation, using default: " << k_e);
  }
  K_e_ << k_e, 0.0, 0.0, k_e;

  for(unsigned int i=0; i<joint_names.size(); i++)
  {
    try
    {
      joints_.push_back(hw->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
  }
  robot_.setLinkLengths(0.9,0.9,0.9,0.95);

  sub_task_space_goal_ = n.subscribe<std_msgs::Float64MultiArray>("goal", 4, &RMRCController::goalCB, this);
  
  return true;
}

void RMRCController::starting(const ros::Time& time)
{
  // initialize desired joint positions
  for(unsigned int i=0; i<joints_.size(); i++) {
      robot_.q[i]  = joints_[i].getPosition();
  }
  robot_.computeForwardKinematics();
}

void RMRCController::update(const ros::Time& t, const ros::Duration& period)
{
  Eigen::Vector2d x_d, dx_d;

  if(spline_.sample(t.toSec(), x_d, dx_d)) {
    Eigen::Vector2d x_EE(robot_.x[4]);
    Eigen::Matrix<double,2,4> J_EE(robot_.J[4]);
    Eigen::Matrix<double,4,2> J_EE_inv;

    Robot::computeMoorePenrosePinv(J_EE, J_EE_inv);

    robot_.dq = J_EE_inv * (K_e_ * (x_d - x_EE) + dx_d);
  }
  else {
    robot_.dq = Eigen::Vector4d::Zero();
  }

  robot_.q += robot_.dq * period.toSec();
  
  robot_.computeForwardKinematics();

  for(int i=0; i<joints_.size(); i++) {
    joints_[i].setCommand(robot_.q(i));
  }
}

void RMRCController::goalCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 2) return;
  
  Eigen::Vector2d x_goal(msg->data[0], msg->data[1]);
    
  double t_start = ros::Time::now().toSec();
  
  spline_.construct(robot_.x[4], t_start, x_goal, t_start + 3.0);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(planar_bot_control::RMRCController, controller_interface::ControllerBase)
