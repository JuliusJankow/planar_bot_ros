#include <planar_bot_planning/prm.h>
#include <cstdlib>

static const double FactorRandom = 2.0/((double)RAND_MAX);

bool PRM::pathCheck(const Node& node1, const Node& node2,
                    const std::vector<PSS_object>& pss_obstacles, 
                    const std::vector<LSS_object>& lss_obstacles, const double resolution)
{
  size_t k = 1;
  Node node_tmp(node1.size());
  double relative_step = resolution/((node2-node1).norm());
  while(1) { // dangerous
    double factor = k*relative_step;
    if(factor >= 1.0) {
      return true;
    }
    node_tmp = node1 + factor * (node2-node1);
    if(collisionCheck(node_tmp, pss_obstacles, lss_obstacles)) {
      return false;
    }
    k++;
  }
  return true;
}

bool PRM::collisionCheck(const Node& node, 
                         const std::vector<PSS_object>& pss_obstacles, 
                         const std::vector<LSS_object>& lss_obstacles)
{
  // create lss objects for robot
  std::array<am_ssv_dist::LSS_object, node.size()> lss_link;
  
  std::array< Eigen::Vector2d, 5 > x;
  std::array< Eigen::Matrix<double,2,4>, 5 > J;
  robot_->computeFK(node,x,J);
  
  // transform ssv models
  for(size_t link=0; link<node.size(); link++) {
    lss_link[link].set_radius(robot_->link_radius_[link]);
    lss_link[link].set_p0(Eigen::Vector4f(x[link](0),   x[link](1),   0.0, 0.0));
    lss_link[link].set_p1(Eigen::Vector4f(x[link+1](0), x[link+1](1), 0.0, 0.0));
  }

  // compute shortest distance for each pair of link-link and link-obstacle
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  
  for(size_t i=0; i<node.size(); i++) {
    for(size_t j=i+2; j<node.size(); j++) {
      dist_calc.cdLL(&(lss_link[i]),&(lss_link[j]),dist_result);
      if(dist_result.distance <= 0.0) {
        return false;
      }
    }
    for(size_t j=0; j<pss_obstacles.size(); j++) {
      dist_calc.cdPL(&(pss_obstacles[j]),&(lss_link[i]),dist_result);
      if(dist_result.distance - pss_obstacles[j].radius() - lss_link[i]->radius() <= 0.0) {
        return false;
      }
    }
    for(size_t j=0; j<lss_obstacles.size(); j++) {
      dist_calc.cdLL(&(lss_obstacles[j]),&(lss_link[i]),dist_result);
      if(dist_result.distance <= 0.0) {
        return false;
      }
    }
  }
}

bool PRM::createRoadmap(const planar_bot::Robot* robot, 
                        const std::vector<PSS_object>& pss_obstacles, 
                        const std::vector<LSS_object>& lss_obstacles, 
                        const PRM_Param& param)
{
  robot_ = robot;
  
  nodes_.clear();
  edges_.clear();
  
  double max_dist_sq = param.maximum_connect_distance*param.maximum_connect_distance;
  double min_dist_sq = param.minimum_sample_distance*param.minimum_sample_distance;
  
  while(nodes_.size() < param.num_nodes) {
    // sample a configuration randomly
    Node node(robot_->num_dof_);
    for(int d=0; d<robot_->num_dof_; d++) {
      // generate random number within joint limits
      node(d) = 0.5 * ( robot_->q_max[d] + robot_->q_min[d] + (robot_->q_max[d] - robot_->q_min[d])*(FactorRandom * rand() - 1.0) );
    }
    
    if(!collisionCheck(node, pss_obstacles, lss_obstacles)) {
      // node is in free space
      
      // collect all neighbours from nodes_
      std::vector<size_t> neighbours;
      bool reject_node = false; // does node have enough clearance to existing nodes
      for(size_t i=0; i<nodes_.size(); i++) {
        double dist_sq = (node-nodes_[i]).squaredNorm();
        if(dist_sq < max_dist_sq) {
          if(dist_sq < min_dist_sq) {
            reject_node = true;
            break;
          }
          neighbours.emplace_back(i);
        }
      }
      
      if(!reject_node) {
        // add node to nodes_ and add edges to neighbours if path is free
        nodes_.emplace_back(node);
        
        for(size_t i=0; i<neighbours.size(); i++) {
          if( pathCheck(node, nodes_[neighbours[i]], pss_obstacles, lss_obstacles, 0.05) ) {
            edges_.emplace_back( Edge(nodes_.size()-1, neighbours[i]) );
          }
        }
      }
    }
  }
}
