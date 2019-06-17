#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>
#include <ssv/ssv_dist_calc.h>

#include "asc_controller.h"

namespace planar_bot_control {

ASCController::ASCController() {}
ASCController::~ASCController() {sub_task_space_goal_.shutdown();}

bool ASCController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
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
  
  double k_e = 0.0;
  if(!n.getParam("drift_compensation", k_e)) {
    ROS_WARN_STREAM("Failed to getParam drift_compensation, using default: " << k_e);
  }
  K_e_ << k_e, 0.0, 0.0, k_e;
  
  if(!n.getParam("k_jla", k_jla_)) {
    ROS_WARN_STREAM("Failed to getParam k_jla, using default: " << k_jla_);
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

  sub_task_space_goal_ = n.subscribe<std_msgs::Float64MultiArray>("goal", 4, &ASCController::goalCB, this);
  
  return true;
}

void ASCController::starting(const ros::Time& time) {
  // initialize desired joint positions
  for(unsigned int i=0; i<joints_.size(); i++) {
      robot_.q[i]  = joints_[i].getPosition();
  }
  robot_.computeForwardKinematics();
}

Eigen::Vector4d ASCController::getGradientJLA() {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);

  for(int i=0; i<4; i++) {
    if(robot_.q(i) < soft_limits_min_[i]) {
      gradient(i) = k_jla_ * (robot_.q(i) - soft_limits_min_[i]) / 
                    ((hard_limits_min_[i] - soft_limits_min_[i]) * (hard_limits_min_[i] - soft_limits_min_[i]));
    }
    if(robot_.q(i) > soft_limits_max_[i]) {
      gradient(i) = k_jla_ * (robot_.q(i) - soft_limits_max_[i]) / 
                    ((hard_limits_max_[i] - soft_limits_max_[i]) * (hard_limits_max_[i] - soft_limits_max_[i]));
    }
  }
  
  return gradient;
}

Eigen::Vector4d ASCController::getGradientCA(std::array<am_ssv_dist::LSS_object, 4>& link_ssv) {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);
  
  am_ssv_dist::PSS_object p1,p2;
  p1.set_p0(Eigen::Vector4f(x1_, y1_, 0.0, 0.0));
  p1.set_radius(radius1_);
  p2.set_p0(Eigen::Vector4f(x2_, y2_, 0.0, 0.0));
  p2.set_radius(radius2_);
  
  for(int link=0; link<4; link++) {
    gradient += computeLinkObstacleGradient(link, &(link_ssv[link]), &p1);
    gradient += computeLinkObstacleGradient(link, &(link_ssv[link]), &p2);
  }
  
  return gradient;
}

Eigen::Vector4d ASCController::computeLinkObstacleGradient(int link, am_ssv_dist::LSS_object* lss, am_ssv_dist::PSS_object* pss) {
  Eigen::Vector4d gradient = Eigen::Vector4d::Zero();

  if(0 > link || link > 3) {
    return gradient;
  }

  // compute shortest distance and corresponding link points
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  dist_calc.cdPL(pss,lss,dist_result);
  double d_rad = dist_result.distance - pss->radius() - lss->radius();
  
  if(d_rad >= d_obs_) {
    return gradient;
  }
  if(d_rad < 0.0) { d_rad = 0.0; }
  
  // compute point jacobians
  Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Vector2d p1_l = p1 - robot_.x[link];
  Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link; column++) {
    J_p1.col(column) << -p1_l(1), p1_l(0);
  }
  J_p1 += robot_.J[link];
  
  // compute gradient
  gradient = m_obs_ * (d_rad - d_obs_) * J_p1.transpose() * (p1 - p0) / dist_result.distance;
  
  return gradient;
}

Eigen::Vector4d ASCController::computeLinkPairGradient(int link_p0, am_ssv_dist::LSS_object* lss0,
                                                       int link_p1, am_ssv_dist::LSS_object* lss1) {
  Eigen::Vector4d gradient = Eigen::Vector4d::Zero();

  if(0 > link_p0 || link_p0 > 3 || 0 > link_p1 || link_p1 > 3) {
    return gradient;
  }

  // compute shortest distance and corresponding link points
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  dist_calc.cdLL(lss0,lss1,dist_result);
  
  if(dist_result.distance >= d_sca_) {
    return gradient;
  }
  
  // compute point jacobians
  Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
  Eigen::Vector2d p0_l = p0 - robot_.x[link_p0];
  Eigen::Matrix<double,2,4> J_p0 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link_p0; column++) {
    J_p0.col(column) << -p0_l(1), p0_l(0);
  }
  J_p0 += robot_.J[link_p0];
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Vector2d p1_l = p1 - robot_.x[link_p1];
  Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link_p1; column++) {
    J_p1.col(column) << -p1_l(1), p1_l(0);
  }
  J_p1 += robot_.J[link_p1];
  
  // compute gradient
  gradient = m_sca_ * (dist_result.distance - d_sca_) * (J_p0.transpose() - J_p1.transpose()) * (p0 - p1) / dist_result.distance;
  
  return gradient;
}

Eigen::Vector4d ASCController::getGradientSCA(std::array<am_ssv_dist::LSS_object, 4>& link_ssv) {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);
  
  gradient += computeLinkPairGradient(0, &(link_ssv[0]), 2, &(link_ssv[2]));
  gradient += computeLinkPairGradient(0, &(link_ssv[0]), 3, &(link_ssv[3]));
  gradient += computeLinkPairGradient(1, &(link_ssv[1]), 3, &(link_ssv[3]));
  
  return gradient;
}

Eigen::Vector4d ASCController::getGradientOfCost() {
  std::array<am_ssv_dist::LSS_object, 4> link_ssv;
  
  // transform ssv models
  for(int link=0; link<4; link++) {
    link_ssv[link].set_radius(0.07);
    link_ssv[link].set_p0(Eigen::Vector4f(robot_.x[link](0),   robot_.x[link](1),   0.0, 0.0));
    link_ssv[link].set_p1(Eigen::Vector4f(robot_.x[link+1](0), robot_.x[link+1](1), 0.0, 0.0));
  }
  
  return getGradientJLA() + getGradientSCA(link_ssv) + getGradientCA(link_ssv);
}

void ASCController::update(const ros::Time& t, const ros::Duration& period) {
  Eigen::Vector2d x_d, dx_d;

  // compute moore-penrose pseudo inverse
  Eigen::Matrix<double,2,4> J_EE(robot_.J[4]);
  Eigen::Matrix<double,4,2> J_EE_inv;
  
  Robot::computeMoorePenrosePinv(J_EE, J_EE_inv);

  Eigen::Vector4d dq_task = Eigen::Vector4d::Zero();
  if(spline_.sample(t.toSec(), x_d, dx_d)) {
      // compute desired joint velocity for task
      dq_task = J_EE_inv * (K_e_ * (x_d - robot_.x[4]) + dx_d);
  }

  // compute Null Space projector
  Eigen::Matrix4d N = Eigen::Matrix4d::Identity() - J_EE_inv * J_EE;

  // map cost function gradient to joint velocity
  Eigen::Vector4d dq_n = -getGradientOfCost();

  robot_.dq = dq_task + N * dq_n;
  robot_.q += robot_.dq * period.toSec();

  for(unsigned int i=0; i<joints_.size(); i++) {
      joints_[i].setCommand(robot_.q[i]);
  }
  
  robot_.computeForwardKinematics();
}

void ASCController::goalCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 2) return;
  
  Eigen::Vector2d x_end(msg->data[0], msg->data[1]);
  
  double t_start = ros::Time::now().toSec() + 0.1;
  
  spline_.construct(robot_.x[4], t_start, x_end, t_start + 3.0);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(planar_bot_control::ASCController, controller_interface::ControllerBase)
