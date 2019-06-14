#include <Eigen/Dense>

#include "reactive_task_tracking_controller.h"
#include "common_global_var.h"
#include "ssv/ssv_dist_calc.h"

namespace planar_bot_control {

static constexpr double damping_jla = 10.0;
Eigen::Vector4d getSubtaskTorqueJLA(const RobotState& robot)
{
  Eigen::Vector4d tau_jla(0.0, 0.0, 0.0, 0.0);

  if(robot.q(0) < q0_min_soft) {
    tau_jla(0) = - 2 * (robot.q(0) - q0_min_soft) / ((q0_min - q0_min_soft) * (q0_min - q0_min_soft)) - damping_jla * robot.dq[0];
  }
  if(robot.q(0) > q0_max_soft) {
    tau_jla(0) = - 2 * (robot.q(0) - q0_max_soft) / ((q0_max - q0_max_soft) * (q0_max - q0_max_soft)) - damping_jla * robot.dq[0];
  }
  for(int i=1; i<4; i++) {
    if(robot.q(i) < -q_abs_soft) {
      tau_jla(i) = - 2 * (robot.q(i) + q_abs_soft) / ((q_abs - q_abs_soft) * (q_abs - q_abs_soft)) - damping_jla * robot.dq[i];
    }
    if(robot.q(i) > q_abs_soft) {
      tau_jla(i) = - 2 * (robot.q(i) - q_abs_soft) / ((q_abs - q_abs_soft) * (q_abs - q_abs_soft)) - damping_jla * robot.dq[i];
    }
  }
  
  return tau_jla;
}

// compute the jacobian mapping from joint space to (x,y) space for arbitrary spots along the robot structure
void computePointJacobian(const RobotState& robot, const Eigen::Vector2d& point, const int link_idx, Eigen::Matrix<double,2,4>& J_point)
{
  Eigen::Vector2d p_rel = point - robot.w[link_idx];
  J_point = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link_idx; column++) {
    J_point.col(column) << -p_rel(1), p_rel(0);
  }
  J_point += robot.J[link_idx];
}

static constexpr double damping_ca = 20.0;
static constexpr double m_ca = 1200.0;
static constexpr double k_rep = 0.8;
Eigen::Vector4d getSubtaskTorqueCA(const RobotState& robot, const int link_idx, 
                                   const am_ssv_dist::LSS_object* lss, const am_ssv_dist::PSS_object* pss)
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
  computePointJacobian(robot, p1, link_idx, J_p1);
  
  // compute subtask jacobian
  Eigen::Vector4d J_pq = J_p1.transpose() * (p1 - p0) / dist_result.distance;
  
  double dd = J_pq.transpose() * robot.dq;
  
  double d_active = std::max(- k_rep * dd, 0.05);
  
  if(d_rad >= d_active/*kD_sca*/) {
    return tau_ca;
  }
  if(d_rad < 0.0) { d_rad = 0.0; }
  
  double dU_dp = m_ca * (d_rad - d_active /*kD_sca*/);
  
  double F_p = - dU_dp - damping_ca * dd;
  
  tau_ca = J_pq * F_p;
  
  return tau_ca;
}

static constexpr double damping_sca = 5.0;
Eigen::Vector4d getSubtaskTorqueSCA(const RobotState& robot, const int link_idx_0, const int link_idx_1,
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
  
  if(dist_result.distance >= kD_sca) {
    return tau_sca;
  }
  
  double dU_dp = kM_sca * (dist_result.distance - kD_sca);
  
  // compute point jacobians
  Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
  Eigen::Matrix<double,2,4> J_p0;
  computePointJacobian(robot, p0, link_idx_0, J_p0);
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Matrix<double,2,4> J_p1;
  computePointJacobian(robot, p1, link_idx_1, J_p1);
  
  // compute subtask jacobian
  Eigen::Vector4d J_pq = (J_p0 - J_p1).transpose() * (p0 - p1) / dist_result.distance;
  
  double F_p = - dU_dp - damping_sca * J_pq.transpose() * robot.dq;
  
  tau_sca = J_pq * F_p;
  
  return tau_sca;
}

} // namespace
