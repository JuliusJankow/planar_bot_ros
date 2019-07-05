#include <planar_bot_planning/prm.h>
#include <ssv/ssv_dist_calc.h>
#include <random>

namespace planar_bot_planning
{

bool PRM::goalCheck(const Node& node, const Eigen::VectorXd& x_goal, const double eps_squared)
{
  std::array< Eigen::Vector2d, 5 > x;
  std::array< Eigen::Matrix<double,2,4>, 5 > J;
  robot_->computeFK(node,x,J);
  
  return ((x[4] - x_goal).squaredNorm()) < eps_squared;
}

double PRM::getCost(const Node& node1, const Node& node2)
{
  std::array< Eigen::Vector2d, 5 > x1,x2;
  std::array< Eigen::Matrix<double,2,4>, 5 > J;
  robot_->computeFK(node1,x1,J);
  robot_->computeFK(node2,x2,J);
  
  return (x1[4] - x2[4]).squaredNorm() + (node1 - node2).squaredNorm();
}

double PRM::getHeuristic(const Node& node, const Eigen::VectorXd& x_goal)
{
  std::array< Eigen::Vector2d, 5 > x;
  std::array< Eigen::Matrix<double,2,4>, 5 > J;
  robot_->computeFK(node,x,J);
  
  return (x[4] - x_goal).squaredNorm();
}

inline double getPathLength(const Path& path)
{
  double result = 0.0;
  for(size_t i=1; i<path.size(); i++) {
    result += (path[i] - path[i-1]).norm();
  }
  return result;
}

bool PRM::searchPath(Path& path, const Eigen::VectorXd& q_start, 
                     const Eigen::VectorXd& x_goal, const double epsilon)
{
  if(nodes_.size() == 0) {
    std::cerr << "No roadmap milestones available!" << std::endl;
    return false;
  }

  if(q_start.size() != nodes_[0].size()) {
    std::cerr << "Dimension of start configuration is different to sampled configurations. Abort search!" << std::endl;
    return false;
  }
  
  // find the node that is closest to q_start
  size_t n_start = 0;
  {
    double squared_distance = (q_start - nodes_[0]).squaredNorm();
    
    for(size_t n=1; n<nodes_.size(); n++) {
      double squared_distance_tmp = (q_start - nodes_[n]).squaredNorm();
      if(squared_distance_tmp < squared_distance) {
        squared_distance = squared_distance_tmp;
        n_start = n;
      }
    }
  }
  
  SearchElement start;
  start.node_idx = n_start;
  start.parent_idx = n_start; // indicating the start point
  start.cost = 0.0;
  
  // initialize search lists
  std::vector<SearchElement> Frontier(1);
  std::vector<SearchElement> Explored(0);
  
  Frontier[0] = start;
  
  double eps_squared = epsilon*epsilon;
  
  while(1) { // repeat until goal found or search failed
    if(Frontier.size() == 0) { // if Frontier is empty, search failed
      std::cerr << "Path search failed: Frontier List is empty!" << std::endl;
      return false;
    }
    
    // find lowest cost in Frontier
    size_t i_node = 0;
    {
      double min_cost = Frontier[0].cost;
      for(size_t i=1; i<Frontier.size(); i++) {
        if(Frontier[i].cost < min_cost) {
          i_node = i;
          min_cost = Frontier[i].cost;
        }
      }
    }
    
    SearchElement node = Frontier[i_node];
    Frontier.erase(Frontier.begin() + i_node);
    Explored.push_back(node);
    
    if(goalCheck(nodes_[node.node_idx], x_goal, eps_squared)) { // found a goal node
      break;
    }
    
    // search for edges of node and add children
    for(size_t i=0; i<edges_.size(); i++) {
      size_t child_idx = 0;
      if(edges_[i][0] == node.node_idx) {
        child_idx = edges_[i][1];
      } else if(edges_[i][1] == node.node_idx) {
        child_idx = edges_[i][0];
      }
      
      if(child_idx) {
        // check if child already visited
        for(size_t j=0; j<Explored.size(); j++) {
          if(Explored[j].node_idx == child_idx) {
            child_idx = 0;
            break;
          }
        }
        
        if(child_idx) {
          SearchElement child;
          child.node_idx = child_idx;
          child.parent_idx = node.node_idx;
          child.cost = node.cost + 
                       getCost(nodes_[child.parent_idx], nodes_[child.node_idx]) +
                       getHeuristic(nodes_[child.node_idx], x_goal);
          
          // check if child already in Frontier
          for(size_t j=0; j<Frontier.size(); j++) {
            if(Frontier[j].node_idx == child.node_idx) {
              if(child.cost < Frontier[j].cost) { // found a cheaper way to child
                Frontier[j].parent_idx = node.node_idx;
                Frontier[j].cost = child.cost;
              }
              child_idx = 0;
              break;
            }
          }
          
          if(child_idx) {
            Frontier.push_back(child);
          }
        }
      }
    }
  }
  
  // The following code is only executed if search was successful
  // Retrieve the path from the Explored List and store configurations
  path.clear();
  
  std::vector<size_t> path_idx(0);
  path_idx.push_back(Explored.back().node_idx);
  path_idx.push_back(Explored.back().parent_idx);
  
  for(size_t i=Explored.size()-2; i>0; i--) {
    if(path_idx.back() == Explored[i].node_idx) {
      path_idx.push_back(Explored[i].parent_idx);
    }
  }
  
  path.push_back(q_start);
  for(size_t i=path_idx.size()-1; i>0; i--) {
    path.push_back(nodes_[path_idx[i]]);
  }
  path.push_back(nodes_[path_idx[0]]);
  
  return true;
}

void PRM::shortcutPath(Path& path, const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                       const std::vector<am_ssv_dist::LSS_object>& lss_obstacles)
{
  // Search for shortcuts along the path
  while(path.size() > 2) {
    size_t q_idx = 0;
    double best_length = getPathLength(path);
    
    // search for way point which should be removed
    for(size_t i=1; i<path.size()-1; i++) {
      if( pathCheck(path[i-1], path[i+1], pss_obstacles, lss_obstacles, 0.05) ) {
        Path path_tmp;
        for(size_t j=0; j<path.size(); j++) {
          // exclude q_i from path_tmp
          if(j != i) {
            path_tmp.push_back(path[j]);
          }
        }
        double tmp_length = getPathLength(path_tmp);
        if(tmp_length < best_length) {
          best_length = tmp_length;
          q_idx = i;
        }
      }
    }
    
    if(q_idx == 0) {
      break;
    }
    
    Path new_path;
    for(size_t i=0; i<path.size(); i++) {
      // exclude q_i from path_tmp
      if(i != q_idx) {
        new_path.push_back(path[i]);
      }
    }
    
    path.swap(new_path);
  }
}

bool PRM::pathCheck(const Node& node1, const Node& node2,
                    const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                    const std::vector<am_ssv_dist::LSS_object>& lss_obstacles, const double resolution)
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
                         const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                         const std::vector<am_ssv_dist::LSS_object>& lss_obstacles)
{
  // create lss objects for robot
  std::vector<am_ssv_dist::LSS_object> lss_link(node.size());
  
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
        return true;
      }
    }
    for(size_t j=0; j<pss_obstacles.size(); j++) {
      dist_calc.cdPL(&(pss_obstacles[j]),&(lss_link[i]),dist_result);
      if(dist_result.distance - pss_obstacles[j].radius() - lss_link[i].radius() <= 0.0) {
        return true;
      }
    }
    for(size_t j=0; j<lss_obstacles.size(); j++) {
      dist_calc.cdLL(&(lss_obstacles[j]),&(lss_link[i]),dist_result);
      if(dist_result.distance <= 0.0) {
        return true;
      }
    }
  }
  
  return false;
}

void PRM::setRoadmap(const std::vector<Node>& nodes, const std::vector<Edge>& edges)
{
  nodes_ = nodes;
  edges_ = edges;
  
  std::cout << "Roadmap consists of " << nodes_.size() << " nodes and " << edges_.size() << " edges." << std::endl;
}

bool PRM::createRoadmap(planar_bot::Robot* robot, 
                        const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                        const std::vector<am_ssv_dist::LSS_object>& lss_obstacles, 
                        const PRM_Param& param)
{
  robot_ = robot;
  
  nodes_.clear();
  edges_.clear();
  
  double max_dist_sq = param.maximum_connect_distance*param.maximum_connect_distance;
  double min_dist_sq = param.minimum_sample_distance*param.minimum_sample_distance;
  
  /* initialize random seed: */
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> dist(-1.0,1.0);
  
  while(nodes_.size() < param.num_nodes) {
    // sample a configuration randomly
    Node node(robot_->num_dof_);
    for(int d=0; d<robot_->num_dof_; d++) {
      // generate random number within joint limits
      node(d) = 0.5 * ( robot_->q_max_[d] + robot_->q_min_[d] + (robot_->q_max_[d] - robot_->q_min_[d])*dist(mt) );
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
        //std::cout << "New Node: " << node.transpose() << std::endl;
        
        for(size_t i=0; i<neighbours.size(); i++) {
          if( pathCheck(node, nodes_[neighbours[i]], pss_obstacles, lss_obstacles, 0.05) ) {
            //std::cout << "New Edge: " << nodes_.size()-1 << " <-> " << neighbours[i] << std::endl;
            Edge edge = {nodes_.size()-1, neighbours[i]};
            edges_.emplace_back(edge);
          }
        }
      }
    }
  }
  
  std::cout << edges_.size() << " Edges found!" << std::endl;
  
  return true;
}

} // namespace
