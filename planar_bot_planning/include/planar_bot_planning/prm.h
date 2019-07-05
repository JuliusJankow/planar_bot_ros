#pragma once

#include <vector>
#include <array>
#include <Eigen/Dense>
#include <ssv/ssv_objects.h>
#include <iostream>
#include <planar_bot.h>

namespace planar_bot_planning
{

using Node = Eigen::VectorXd;
using Edge = std::array<size_t, 2>;
using Path = std::vector< Eigen::VectorXd >;

struct PRM_Param
{
  int num_nodes; // desired number of samples from free c-space
  double minimum_sample_distance; // min distance with which the samples are placed
  double maximum_connect_distance; // max distance for which samples can be connected
};

struct SearchElement
{
  size_t node_idx;
  size_t parent_idx;
  double cost;
};

class PRM
{
public:

  PRM() {}
  
  PRM(planar_bot::Robot* robot, const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
      const std::vector<am_ssv_dist::LSS_object>& lss_obstacles, const PRM_Param& param) {
    createRoadmap(robot, pss_obstacles, lss_obstacles, param); }
    
  PRM(planar_bot::Robot* robot, const std::vector<Node>& nodes, 
      const std::vector<Edge>& edges) : robot_(robot), nodes_(nodes), edges_(edges) {
      std::cout << "Roadmap consists of " << nodes_.size() << " nodes and " << edges_.size() << " edges." << std::endl; }
  
  bool createRoadmap(planar_bot::Robot* robot, const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                     const std::vector<am_ssv_dist::LSS_object>& lss_obstacles, const PRM_Param& param);
                     
  void setRoadmap(const std::vector<Node>& nodes, const std::vector<Edge>& edges);
                     
  void getRoadmap(std::vector<Node>& nodes, std::vector<Edge>& edges) {
    nodes = nodes_; edges = edges_; }
                     
  bool searchPath(Path& path, const Eigen::VectorXd& q_start, 
                  const Eigen::VectorXd& x_goal, const double epsilon);
                  
  void shortcutPath(Path& path, const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                    const std::vector<am_ssv_dist::LSS_object>& lss_obstacles);

private:

  double getCost(const Node& node1, const Node& node2);
  
  double getHeuristic(const Node& node, const Eigen::VectorXd& x_goal);

  bool goalCheck(const Node& node, const Eigen::VectorXd& x_goal, const double eps_squared);

  bool pathCheck(const Node& node1, const Node& node2,
                 const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                 const std::vector<am_ssv_dist::LSS_object>& lss_obstacles, const double resolution);
                 
  bool collisionCheck(const Node& node, 
                      const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                      const std::vector<am_ssv_dist::LSS_object>& lss_obstacles);

  std::vector<Node> nodes_;
  std::vector<Edge> edges_;

  planar_bot::Robot* robot_;

};

} // namespace
