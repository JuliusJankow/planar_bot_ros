#include <planar_bot_planning/elastic_band.h>
#include <planar_bot.h>
#include <ssv/ssv_dist_calc.h>
#include <iostream>
#include <cmath>

using namespace planar_bot_planning;

double ElasticBand::update(Path& path, const Param& param, 
                           const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                           const std::vector<am_ssv_dist::LSS_object>& lss_obstacles)
{
  double cost = 0.0;
  double rest_length = param.relative_rest_length * param.stride;
  std::vector<Eigen::VectorXd> F(path.size()-2);
  for(size_t k=1; k<path.size()-1; k++) {
    // compute internal contraction force
    double norm1 = (path[k] - path[k-1]).norm();
    double norm2 = (path[k] - path[k+1]).norm();
    Eigen::VectorXd tension1 = ((norm1 - rest_length)/norm1) * (path[k] - path[k-1]);
    Eigen::VectorXd tension2 = ((norm2 - rest_length)/norm2) * (path[k] - path[k+1]);
    F[k-1] = - param.spring_constant * (tension1 + tension2);
    
    // compute external obstacle force (including jla and sca)
    Eigen::VectorXd F_obs(robot_->num_dof_);
    getObstacleForce(F_obs, path[k], param, pss_obstacles, lss_obstacles);
    F[k-1] += F_obs;
    
    cost += F[k-1].norm();
  }
  for(size_t k=1; k<path.size()-1; k++) {
    path[k] += param.lambda * F[k-1];
  }
  
  return cost / (path.size()-2);
}

void ElasticBand::preparePath(Path& path, const Param& param)
{
  Path result;
  
  // compute path with correct stride
  for(size_t n=0; n<path.size()-1; n++) {
    double step = param.stride / (path[n+1] - path[n]).norm();
    
    double factor = 0.0;
    while(factor < 1.0) {
      result.emplace_back(path[n] + factor * (path[n+1] - path[n]));
      factor += step;
    }
  }
  
  path.swap(result);
}

bool ElasticBand::smooth(Path& result, const Path& initial_path, const Param& param, 
                         const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                         const std::vector<am_ssv_dist::LSS_object>& lss_obstacles)
{
  result.clear();
  result = initial_path;
  
  double cost = update(result, param, pss_obstacles, lss_obstacles);
  
  // iterate over path in order to smooth it
  for(size_t iter=0; iter<param.num_iterations; iter++) {
    double new_cost = update(result, param, pss_obstacles, lss_obstacles);
    if(std::isnan(cost)) {
      //std::cerr << "Elastic Band diverged after " << iter+2 << " iteraions." << std::endl;
      return false;
    }
    if(std::abs(new_cost - cost) * 100000.0 < 0.001) {
      //std::cout << "Elastic Band converged after " << iter+2 << " iteraions." << std::endl;
      return true;
    }
    cost = new_cost;
  }
  
  return true;
}

void ElasticBand::getObstacleForce(Eigen::VectorXd& F_obs, const Eigen::VectorXd& q, const Param& param,
                                   const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                                   const std::vector<am_ssv_dist::LSS_object>& lss_obstacles)
{
  // create lss objects for robot
  std::vector<am_ssv_dist::LSS_object> lss_link(q.size());
  
  std::array< Eigen::Vector2d, 5 > x;
  std::array< Eigen::Matrix<double,2,4>, 5 > J;
  robot_->computeFK(q,x,J);
  
  // transform ssv models
  for(size_t link=0; link<q.size(); link++) {
    lss_link[link].set_radius(robot_->link_radius_[link]);
    lss_link[link].set_p0(Eigen::Vector4f(x[link](0),   x[link](1),   0.0, 0.0));
    lss_link[link].set_p1(Eigen::Vector4f(x[link+1](0), x[link+1](1), 0.0, 0.0));
  }

  // compute shortest distance for each pair of link-link and link-obstacle
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  
  F_obs = Eigen::VectorXd::Zero(robot_->num_dof_);
  double gain = 0.01 * param.spring_constant;
  
  for(size_t i=0; i<robot_->num_dof_; i++) {
    // jla
    if(q(i) < robot_->q_min_[i]) {
      F_obs(i) += - gain * (q(i) - robot_->q_min_[i]);
    }
    if(q(i) > robot_->q_max_[i]) {
      F_obs(i) += - gain * (q(i) - robot_->q_max_[i]);
    }
    // sca
    for(size_t j=i+2; j<robot_->num_dof_; j++) {
      dist_calc.cdLL(&(lss_link[i]),&(lss_link[j]),dist_result);
      if(dist_result.distance <= param.d_active) {
        Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
        Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
        
        Eigen::Vector2d p0_rel = p0 - x[i];
        Eigen::Matrix<double,2,4> J_p0 = Eigen::Matrix<double,2,4>::Zero();
        for(int column=0; column<=i; column++) {
          J_p0.col(column) << -p0_rel(1), p0_rel(0);
        }
        J_p0 += J[i];
        
        Eigen::Vector2d p1_rel = p1 - x[j];
        Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
        for(int column=0; column<=j; column++) {
          J_p1.col(column) << -p1_rel(1), p1_rel(0);
        }
        J_p1 += J[j];
    
        // compute avoidance jacobian
        Eigen::Vector4d J_sca = (J_p0 - J_p1).transpose() * (p0 - p1) / 
                        (dist_result.distance + lss_link[i].radius() + lss_link[j].radius());
        double f = - gain * (dist_result.distance - param.d_active);
        
        F_obs += J_sca * f;
      }
    }
    // ca
    for(size_t j=0; j<pss_obstacles.size(); j++) {
      dist_calc.cdPL(&(pss_obstacles[j]),&(lss_link[i]),dist_result);
      if(dist_result.distance - pss_obstacles[j].radius() - lss_link[i].radius() <= param.d_active) {
        Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
        Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
        
        Eigen::Vector2d p1_rel = p1 - x[i];
        Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
        for(int column=0; column<=i; column++) {
          J_p1.col(column) << -p1_rel(1), p1_rel(0);
        }
        J_p1 += J[i];
    
        // compute avoidance jacobian
        Eigen::Vector4d J_sca = J_p1.transpose() * (p1 - p0) / dist_result.distance;
        double f = - gain * (dist_result.distance - pss_obstacles[j].radius() - lss_link[i].radius() - param.d_active);
        
        F_obs += J_sca * f;
      }
    }
    // ca
    for(size_t j=0; j<lss_obstacles.size(); j++) {
      dist_calc.cdLL(&(lss_obstacles[j]),&(lss_link[i]),dist_result);
      if(dist_result.distance <= param.d_active) {
        Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
        Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
        
        Eigen::Vector2d p1_rel = p1 - x[i];
        Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
        for(int column=0; column<=i; column++) {
          J_p1.col(column) << -p1_rel(1), p1_rel(0);
        }
        J_p1 += J[i];
    
        // compute avoidance jacobian
        Eigen::Vector4d J_sca = J_p1.transpose() * (p1 - p0) / 
          (dist_result.distance + lss_link[i].radius() + lss_link[j].radius());
        double f = - gain * (dist_result.distance - param.d_active);
        
        F_obs += J_sca * f;
      }
    }
  }
}
