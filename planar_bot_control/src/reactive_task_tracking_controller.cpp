#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>
#include <ssv/ssv_dist_calc.h>

#include "reactive_task_tracking_controller.h"

namespace planar_bot_control {

RTTController::RTTController() {}
RTTController::~RTTController() {sub_task_space_goal_.shutdown();}

bool RTTController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
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
  
  param_name = "hard_limits_min";
  if(!n.getParam(param_name, hard_limits_min_)) {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }
  param_name = "hard_limits_max";
  if(!n.getParam(param_name, hard_limits_max_)) {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }
  param_name = "soft_limits_min";
  if(!n.getParam(param_name, soft_limits_min_)) {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }
  param_name = "soft_limits_max";
  if(!n.getParam(param_name, soft_limits_max_)) {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }
  
  if(hard_limits_min_.size() + hard_limits_max_.size() + soft_limits_min_.size() + soft_limits_max_.size()
      != 4*joint_names.size()) {
    ROS_ERROR_STREAM("List of joint limits is incomplete.");
    return false;
  }
  
  double k_p = 0.0;
  if(!n.getParam("task_space_p_gain", k_p)) {
    ROS_WARN_STREAM("Failed to getParam task_space_error_feedback_gain, using default: " << k_p);
  }
  K_p_ << k_p, 0.0, 0.0, k_p;
  
  double k_d = 2.0*std::sqrt(k_p);
  K_d_ << k_d, 0.0, 0.0, k_d;
  
  if(!n.getParam("k_jla", k_jla_)) {
    ROS_WARN_STREAM("Failed to getParam k_jla, using default: " << k_jla_);
  }
  if(!n.getParam("d_jla", d_jla_)) {
    ROS_WARN_STREAM("Failed to getParam d_jla, using default: " << d_jla_);
  }
  if(!n.getParam("d_sca", d_sca_)) {
    ROS_WARN_STREAM("Failed to getParam d_sca, using default: " << d_sca_);
  }
  if(!n.getParam("m_sca", m_sca_)) {
    ROS_WARN_STREAM("Failed to getParam m_sca, using default: " << m_sca_);
  }
  if(!n.getParam("d_obs", d_obs_)) {
    ROS_WARN_STREAM("Failed to getParam d_obs, using default: " << d_obs_);
  }
  if(!n.getParam("m_obs", m_obs_)) {
    ROS_WARN_STREAM("Failed to getParam m_obs, using default: " << m_obs_);
  }
  if(!n.getParam("inflation", inflation_factor_)) {
    ROS_WARN_STREAM("Failed to getParam inflation, using default: " << inflation_factor_);
  }
  if(!n.getParam("time_scaling_upper_thresh", time_scaling_upper_thresh_)) {
    ROS_WARN_STREAM("Failed to getParam time_scaling_upper_thresh, using default: " << time_scaling_upper_thresh_);
  }
  if(!n.getParam("time_scaling_lower_thresh", time_scaling_lower_thresh_)) {
    ROS_WARN_STREAM("Failed to getParam time_scaling_lower_thresh, using default: " << time_scaling_lower_thresh_);
  }
  if(!n.getParam("joint_damping", joint_damping_)) {
    ROS_WARN_STREAM("Failed to getParam joint_damping, using default: " << joint_damping_);
  }
  if(!n.getParam("minimum_active_distance", minimum_active_distance_)) {
    ROS_WARN_STREAM("Failed to getParam minimum_active_distance, using default: " << minimum_active_distance_);
  }
  if(!n.getParam("damping_sca", damping_sca_)) {
    ROS_WARN_STREAM("Failed to getParam damping_sca, using default: " << damping_sca_);
  }

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

  sub_task_space_goal_ = n.subscribe<std_msgs::Float64MultiArray>("goal", 4, &RTTController::goalCB, this);
  return true;
}

void RTTController::starting(const ros::Time& time) {
  // initialize desired joint positions
  robot_.q[0] = 1.7;
  robot_.q[1] = -0.5;
  robot_.q[2] = -0.5;
  robot_.q[3] = 0.3;
  
  robot_.computeForwardKinematics();
  x_des = robot_.x[4];
  dx_des << 0.0, 0.0;
}

Eigen::Vector4d RTTController::getTauFromSubtasks()
{
  std::array<am_ssv_dist::LSS_object, 4> link_ssv;
  
  // transform ssv models
  for(int link=0; link<4; link++) {
    link_ssv[link].set_radius(0.07);
    link_ssv[link].set_p0(Eigen::Vector4f(robot_.x[link](0),   robot_.x[link](1),   0.0, 0.0));
    link_ssv[link].set_p1(Eigen::Vector4f(robot_.x[link+1](0), robot_.x[link+1](1), 0.0, 0.0));
  }
  
  // create obstacle ssv models
  am_ssv_dist::PSS_object p1,p2;
  p1.set_p0(Eigen::Vector4f(x1_, y1_, 0.0, 0.0));
  p1.set_radius(radius1_);
  p2.set_p0(Eigen::Vector4f(x2_, y2_, 0.0, 0.0));
  p2.set_radius(radius2_);
  
  Eigen::Vector4d tau_sub(Eigen::Vector4d::Zero());
  
  // compute joint limit avoidance (JLA) torques
  tau_sub += getSubtaskTorqueJLA();
  
  // compute collision avoidance (CA) torques
  for(int link=0; link<4; link++) {
    tau_sub += getSubtaskTorqueCA(link, &(link_ssv[link]), &p1);
    tau_sub += getSubtaskTorqueCA(link, &(link_ssv[link]), &p2);
  }
  
  // compute self collision avoidance (SCA) torques
  tau_sub += getSubtaskTorqueSCA(0, 2, &(link_ssv[0]), &(link_ssv[2]));
  tau_sub += getSubtaskTorqueSCA(0, 3, &(link_ssv[0]), &(link_ssv[3]));
  tau_sub += getSubtaskTorqueSCA(1, 3, &(link_ssv[1]), &(link_ssv[3]));
  
  return tau_sub;
}

void RTTController::update(const ros::Time& t, const ros::Duration& period) {
  for(unsigned int i=0; i<joints_.size(); i++) {
    robot_.q[i]  = joints_[i].getPosition();
    robot_.dq[i] = joints_[i].getVelocity();
  }

  // update forward kinematics and jacobian
  robot_.computeForwardKinematics();

  Eigen::Vector2d x(robot_.x[4]);
  Eigen::Matrix<double,2,4> J_EE(robot_.J[4]);
  
  Eigen::Vector2d x_d, dx_d;
  if(spline_.sample(trajectory_time_, x_d, dx_d)) {
    Eigen::Vector2d e_x = x - x_d;
    
    double time_scale = (time_scaling_upper_thresh_ - e_x.norm()) / 
                        (time_scaling_upper_thresh_ - time_scaling_lower_thresh_);
    time_scale = std::max(0.0, std::min(1.0, time_scale));
    
    x_des = x_d;
    dx_des = time_scale * dx_d;
    
    trajectory_time_ += time_scale * period.toSec();
  }
  Eigen::Vector2d dx(J_EE*robot_.dq);
  
  // compute task space control law
  Eigen::Vector2d F_task = - K_p_ * (x - x_des) - K_d_ * (dx - dx_des);

  // compute torque to locally optimize subtasks
  Eigen::Vector4d tau_pot = getTauFromSubtasks();

  Eigen::Vector4d tau_c = J_EE.transpose() * F_task + tau_pot - joint_damping_ * robot_.dq;

  double tau_max = 80.0;
  double multiplicator = 1.0;
  for(unsigned int i=0; i<joints_.size(); i++) {
    double tau_abs = std::abs(tau_c[i]);
    if(tau_abs > tau_max) multiplicator = std::min(multiplicator, tau_max/tau_abs);
  }
  for(unsigned int i=0; i<joints_.size(); i++) {
    joints_[i].setCommand(tau_c[i]*multiplicator);
  }
}

void RTTController::goalCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 2) return;
  
  Eigen::Vector2d x_goal(msg->data[0], msg->data[1]);
  
  double t_start = 0.0;
  
  spline_.construct(robot_.x[4], t_start, x_goal, t_start + 3.0);
  
  trajectory_time_ = t_start;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(planar_bot_control::RTTController, controller_interface::ControllerBase)
