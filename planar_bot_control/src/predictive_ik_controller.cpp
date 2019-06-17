#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>
#include <ssv/ssv_dist_calc.h>

#include "predictive_ik_controller.h"

namespace planar_bot_control {

PIKController::PIKController() {}
PIKController::~PIKController() {sub_task_space_goal_.shutdown();}

bool PIKController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
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
  if(!n.getParam("T_horizon", t_horizon_)) {
    ROS_WARN_STREAM("Failed to getParam T_horizon, using default: " << t_horizon_);
  }
  if(!n.getParam("k_q", k_q_)) {
    ROS_WARN_STREAM("Failed to getParam k_q, using default: " << k_q_);
  }
  if(!n.getParam("k_dq", k_dq_)) {
    ROS_WARN_STREAM("Failed to getParam k_dq, using default: " << k_dq_);
  }
  if(!n.getParam("k_du", k_du_)) {
    ROS_WARN_STREAM("Failed to getParam k_du, using default: " << k_du_);
  }
  if(!n.getParam("alpha_opt", alpha_opt_)) {
    ROS_WARN_STREAM("Failed to getParam alpha_opt, using default: " << alpha_opt_);
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

  sub_task_space_goal_ = n.subscribe<std_msgs::Float64MultiArray>("goal", 4, &PIKController::goalCB, this);
  
  return true;
}

void PIKController::starting(const ros::Time& time) {
  // initialize desired joint positions
  for(unsigned int i=0; i<joints_.size(); i++) {
    robot_.q[i]  = joints_[i].getPosition();
    u_[i] = 0.0;
  }
  robot_.computeForwardKinematics();
}

Eigen::Vector4d PIKController::getGradientJLA(const Robot& robot) {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);
  for(int i=0; i<4; i++) {
    if(robot.q(i) < soft_limits_min_[i]) {
      gradient(i) = k_jla_ * (robot.q(i) - soft_limits_min_[i]) / 
                    ((hard_limits_min_[i] - soft_limits_min_[i]) * (hard_limits_min_[i] - soft_limits_min_[i]));
    }
    if(robot.q(i) > soft_limits_max_[i]) {
      gradient(i) = k_jla_ * (robot.q(i) - soft_limits_max_[i]) / 
                    ((hard_limits_max_[i] - soft_limits_max_[i]) * (hard_limits_max_[i] - soft_limits_max_[i]));
    }
  }
  return gradient;
}

Eigen::Vector4d PIKController::computeLinkObstacleGradient(const Robot& robot, const int link, 
                            const am_ssv_dist::LSS_object* lss, const am_ssv_dist::PSS_object* pss) {
  Eigen::Vector4d gradient = Eigen::Vector4d::Zero();

  if(0 > link || link > 3) {
    return gradient;
  }

  // compute shortest distance and corresponding link points
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  dist_calc.cdPL(pss,lss,dist_result);
  double d_rad = dist_result.distance - pss->radius() - lss->radius();

  if(d_rad >= d_obs_ || std::abs(dist_result.distance) < 0.01) {
    return gradient;
  }
  if(d_rad < 0.0) { d_rad = 0.0; }
  
  // compute point jacobians
  Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Vector2d p1_l = p1 - robot.x[link];
  Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link; column++) {
    J_p1.col(column) << -p1_l(1), p1_l(0);
  }
  J_p1 += robot.J[link];
  
  // compute gradient
  gradient = m_obs_ * (d_rad - d_obs_) * J_p1.transpose() * (p1 - p0) / dist_result.distance;
  
  return gradient;
}

Eigen::Vector4d PIKController::computeLinkPairGradient(const Robot& robot,
                                                       const int link_p0, const am_ssv_dist::LSS_object* lss0,
                                                       const int link_p1, const am_ssv_dist::LSS_object* lss1) {
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
  Eigen::Vector2d p0_l = p0 - robot.x[link_p0];
  Eigen::Matrix<double,2,4> J_p0 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link_p0; column++) {
    J_p0.col(column) << -p0_l(1), p0_l(0);
  }
  J_p0 += robot.J[link_p0];
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Vector2d p1_l = p1 - robot.x[link_p1];
  Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link_p1; column++) {
    J_p1.col(column) << -p1_l(1), p1_l(0);
  }
  J_p1 += robot.J[link_p1];
  
  // compute gradient
  gradient = m_sca_ * (dist_result.distance - d_sca_) * (J_p0.transpose() - J_p1.transpose()) * (p0 - p1) / dist_result.distance;
  
  return gradient;
}

Eigen::Vector4d PIKController::getGradientCA(const Robot& robot, const std::array<am_ssv_dist::LSS_object, 4>& link_ssv) {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);
  
  am_ssv_dist::PSS_object p1,p2;
  p1.set_p0(Eigen::Vector4f(x1_, y1_, 0.0, 0.0));
  p1.set_radius(radius1_);
  p2.set_p0(Eigen::Vector4f(x2_, y2_, 0.0, 0.0));
  p2.set_radius(radius2_);
  
  for(int link=0; link<4; link++) {
    gradient += computeLinkObstacleGradient(robot, link, &(link_ssv[link]), &p1);
    gradient += computeLinkObstacleGradient(robot, link, &(link_ssv[link]), &p2);
  }
  
  return gradient;
}

Eigen::Vector4d PIKController::getGradientSCA(const Robot& robot, const std::array<am_ssv_dist::LSS_object, 4>& link_ssv) {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);
  
  gradient += computeLinkPairGradient(robot, 0, &(link_ssv[0]), 2, &(link_ssv[2]));
  gradient += computeLinkPairGradient(robot, 0, &(link_ssv[0]), 3, &(link_ssv[3]));
  gradient += computeLinkPairGradient(robot, 1, &(link_ssv[1]), 3, &(link_ssv[3]));
  
  return gradient;
}

Eigen::Vector4d PIKController::getGradientOfCost(const Robot& robot) {
  std::array<am_ssv_dist::LSS_object, 4> link_ssv;
  
  // transform ssv models
  for(int link=0; link<4; link++) {
    link_ssv[link].set_radius(0.07);
    link_ssv[link].set_p0(Eigen::Vector4f(robot.x[link](0),   robot.x[link](1),   0.0, 0.0));
    link_ssv[link].set_p1(Eigen::Vector4f(robot.x[link+1](0), robot.x[link+1](1), 0.0, 0.0));
  }
  
  Eigen::Vector4d gradient = getGradientJLA(robot) + getGradientSCA(robot, link_ssv) + getGradientCA(robot, link_ssv);
  
  return gradient;
}

Eigen::Matrix4d calculateHessFq(const Robot& robot, const Eigen::Matrix<double,4,2>& J_inv, 
                                const Eigen::Vector2d& dx, const Eigen::Vector4d& u)
{
  Eigen:Matrix4d HessFq;
  Eigen::Matrix<double,2,4> J(robot.J[4]);
  
  for(int k=0; k<4; k++) { // for each joint k as second derivative
    Eigen::Matrix<double,2,4> Jqk;
    for(int j=0; j<4; j++) { // for each joint j as first derivative
      int idx = std::max(j,k);
      Jqk(0,j) = -J(1,idx); // G1(idx);
      Jqk(1,j) =  J(0,idx); // G2(idx);
    }
    Eigen::Matrix4d J_inv_times_jqk = J_inv * Jqk;
    Eigen::Matrix<double,4,2> J_inv_qk = ((J_inv_times_jqk * (Eigen::Matrix4d::Identity() - J_inv*J)).transpose() - J_inv_times_jqk) * J_inv;
    HessFq.col(k) = J_inv_qk * dx + (Eigen::Matrix4d::Identity() - J_inv_qk * J - J_inv_times_jqk) * u;
  }
  
  return HessFq;
}

Eigen::Vector4d PIKController::getOptimizedNullSpaceCommand(const Eigen::Vector2d& dx, const double& sample_time)
{
  Robot predicted_robot;
  predicted_robot.setLinkLengths(0.9,0.9,0.9,0.95);
  
  Eigen::Vector4d u, lambda1, dlambda1, lambda2, dlambda2;
  Eigen::Matrix<double,2,4> J;
  Eigen::Matrix<double,4,2> J_inv;
  std::vector<Eigen::Matrix4d> N(t_horizon_), hess_fq(t_horizon_);
  std::vector<Eigen::Vector4d> du(t_horizon_), dq(t_horizon_), grad_H(t_horizon_);
  
  for(int k=0; k<t_horizon_; k++) {
    du[k] = Eigen::Vector4d::Zero();
  }
  
  for(int i=0; i<num_opt_iter_; i++) {
    predicted_robot.q = robot_.q;
    predicted_robot.computeForwardKinematics();
    u = u_;
    for(int k=0; k<t_horizon_; k++) { // forward simulation of system
      // forward/inverse kinematics
      J = predicted_robot.J[4];
      Robot::computeMoorePenrosePinv(J, J_inv);
      N[k] = Eigen::Matrix4d::Identity() - J_inv * J;
      
      // numeric integration
      dq[k] = J_inv * dx + N[k] * u;
      predicted_robot.q += sample_time * dq[k];
      u += sample_time * du[k];
      
      predicted_robot.computeForwardKinematics();
      
      // calculation of current gradients
      hess_fq[k] = calculateHessFq(predicted_robot, J_inv, dx, u);
      grad_H[k] = getGradientOfCost(predicted_robot);
    }

    lambda1 = Eigen::Vector4d(Eigen::Vector4d::Zero());
    lambda2 = lambda1;
    for(int k=t_horizon_-1; k>=0; k--) { // backward simulation of lambda de's
      dlambda1 = - hess_fq[k].transpose() * (lambda1 + k_dq_ * dq[k]) - k_q_ * grad_H[k];
      dlambda2 = - N[k] * (lambda1 + k_dq_ * dq[k]);
      lambda1 = lambda1 - sample_time * dlambda1;
      lambda2 = lambda2 - sample_time * dlambda2;
      
      du[k] = du[k] - alpha_opt_ * (k_du_ * du[k] + lambda2);
    }
  }
  
  return du[0];
}

void PIKController::update(const ros::Time& t, const ros::Duration& period) {
  double sample_time = period.toSec();

  Eigen::Vector2d x_d, dx_d(0.0,0.0);
  Eigen::Vector4d dq_task(0.0, 0.0, 0.0, 0.0);

  // update forward kinematics and jacobian
  Eigen::MatrixXd J(robot_.J[4]);

  // compute moore-penrose pseudo inverse
  Eigen::Matrix<double,4,2> J_inv;
  Robot::computeMoorePenrosePinv(J, J_inv);
  // compute Null Space projector
  Eigen::Matrix4d N = Eigen::Matrix4d::Identity() - J_inv * J;

  if(spline_.sample(t.toSec(), x_d, dx_d)) {
    // compute desired joint velocity for task
    dq_task = J_inv * (K_e_ * (x_d - robot_.x[4]) + dx_d);
  }

  // optimize null space acceleration for a fixed horizon
  u_ = u_ + sample_time * getOptimizedNullSpaceCommand(dx_d, sample_time);

  robot_.dq = dq_task + N * u_;
  robot_.q += robot_.dq * sample_time;

  for(unsigned int i=0; i<joints_.size(); i++) {
    joints_[i].setCommand(robot_.q[i]);
  }

  robot_.computeForwardKinematics();
  
  last_update_time_ = t;
}

void PIKController::goalCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 2) return;
  
  Eigen::Vector2d x_end(msg->data[0], msg->data[1]);
  
  double t_start = last_update_time_.toSec() + 0.1;
  
  spline_.construct(robot_.x[4], t_start, x_end, t_start + 3.0);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(planar_bot_control::PIKController, controller_interface::ControllerBase)
