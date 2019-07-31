#include <Eigen/Dense>
#include <kinematics/kinematics.h>
#include <ssv/ssv_dist_calc.h>

#include "reactive_task_tracking_controller.h"

namespace planar_bot_control {

Eigen::Vector4d RTTController::getSubtaskTorqueJLA()
{
  Eigen::Vector4d tau_jla(0.0, 0.0, 0.0, 0.0);

  for(int i=0; i<4; i++) {
    if(robot_.q(i) < soft_limits_min_[i]) {
      tau_jla(i) = - k_jla_ * (robot_.q(i) - soft_limits_min_[i]) / 
                    ((hard_limits_min_[i] - soft_limits_min_[i]) * (hard_limits_min_[i] - soft_limits_min_[i])) - d_jla_ * robot_.dq(i);
    }
    if(robot_.q(i) > soft_limits_max_[i]) {
      tau_jla(i) = - k_jla_ * (robot_.q(i) - soft_limits_max_[i]) / 
                    ((hard_limits_max_[i] - soft_limits_max_[i]) * (hard_limits_max_[i] - soft_limits_max_[i])) - d_jla_ * robot_.dq(i);
    }
  }
  
  return tau_jla;
}

// compute the jacobian mapping from joint space to (x,y) space for arbitrary spots along the robot structure
void computePointJacobian(const Robot& robot, const Eigen::Vector2d& point, const int link_idx, Eigen::Matrix<double,2,4>& J_point)
{
  Eigen::Vector2d p_rel = point - robot.x[link_idx];
  J_point = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link_idx; column++) {
    J_point.col(column) << -p_rel(1), p_rel(0);
  }
  J_point += robot.J[link_idx];
}

Eigen::Vector4d RTTController::getSubtaskTorqueCA(const int link_idx,
    const am_ssv_dist::LSS_object* lss, const am_ssv_dist::PSS_object* pss, double& d_active)
{
  Eigen::Vector4d tau_ca = Eigen::Vector4d::Zero();

  if(0 > link_idx || link_idx > 3) {
    return tau_ca;
  }

  // compute shortest distance and corresponding link points
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  dist_calc.cdPL(pss,lss,dist_result);
  double d_rad = dist_result.distance - pss->radius() - lss->radius();
  
  // compute point jacobian
  Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Matrix<double,2,4> J_p1;
  computePointJacobian(robot_, p1, link_idx, J_p1);
  
  // compute subtask jacobian
  Eigen::Vector4d J_pq = J_p1.transpose() * (p1 - p0) / dist_result.distance;
  
  double dd = J_pq.transpose() * robot_.dq;
  
  d_active = std::max(- inflation_factor_ * dd, minimum_active_distance_);
  
  if(d_rad >= d_active) {
    d_active = 0.0;
    return tau_ca;
  }
  if(d_rad < 0.0) { d_rad = 0.0; }
  
  double dU_dp = m_obs_ * (d_rad - d_active);
  
  double F_p = - dU_dp - d_obs_ * dd;
  
  tau_ca = J_pq * F_p;
  
  return tau_ca;
}

Eigen::Vector4d RTTController::getSubtaskTorqueSCA(const int link_idx_0, const int link_idx_1,
                                    const am_ssv_dist::LSS_object* lss_0, const am_ssv_dist::LSS_object* lss_1)
{
  Eigen::Vector4d tau_sca = Eigen::Vector4d::Zero();

  if(0 > link_idx_0 || link_idx_0 > 3 || 0 > link_idx_1 || link_idx_1 > 3) {
    return tau_sca;
  }

  // compute shortest distance and corresponding link points
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  dist_calc.cdLL(lss_0,lss_1,dist_result);
  
  if(dist_result.distance >= d_sca_) {
    return tau_sca;
  }
  
  double dU_dp = m_sca_ * (dist_result.distance - d_sca_);
  
  // compute point jacobians
  Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
  Eigen::Matrix<double,2,4> J_p0;
  computePointJacobian(robot_, p0, link_idx_0, J_p0);
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Matrix<double,2,4> J_p1;
  computePointJacobian(robot_, p1, link_idx_1, J_p1);
  
  // compute subtask jacobian
  Eigen::Vector4d J_pq = (J_p0 - J_p1).transpose() * (p0 - p1) / dist_result.distance;
  
  double F_p = - dU_dp - damping_sca_ * J_pq.transpose() * robot_.dq;
  
  tau_sca = J_pq * F_p;
  
  return tau_sca;
}

} // namespace
