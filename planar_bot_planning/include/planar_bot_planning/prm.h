#pragma once

#include <vector>
#include <Eigen/Dense>
#include <ssv/ssv_objects.h>

namespace planar_bot_planning
{

using Node = Eigen::VectorXd;
using Edge = std::array<unsigned int, 2>;

struct PRM_Param
{
  int num_nodes; // desired number of samples from free c-space
  double minimum_sample_distance; // min distance with which the samples are placed
  double maximum_connect_distance; // max distance for which samples can be connected
};

class PRM
{
public:

  PRM() {}
  
  PRM(const planar_bot::Robot* robot, const std::vector<PSS_object>& pss_obstacles, 
      const std::vector<LSS_object>& lss_obstacles, const PRM_Param& param) {
    createRoadmap(robot, pss_obstacles, lss_obstacles, param); }
    
  PRM(const planar_bot::Robot* robot, const std::vector<Node>& nodes, 
      const std::vector<Edge>& edges) : robot_(robot), nodes_(nodes), edges_(edges) {}
  
  bool createRoadmap(const planar_bot::Robot* robot, const std::vector<PSS_object>& pss_obstacles, 
                     const std::vector<LSS_object>& lss_obstacles, const PRM_Param& param);
                     
  bool searchPath(Path& path, const std::vector<double> q_start, 
                  const std::vector<double> x_goal, const double epsilon);

private:

  bool pathCheck(const Node& node1, const Node& node2,
                 const std::vector<PSS_object>& pss_obstacles, 
                 const std::vector<LSS_object>& lss_obstacles, const double resolution);
                 
  bool collisionCheck(const Node& node, 
                      const std::vector<PSS_object>& pss_obstacles, 
                      const std::vector<LSS_object>& lss_obstacles);

  std::vector<Node> nodes_;
  std::vector<Edge> edges_;

  planar_bot::Robot* robot_;

};

} // namespace
