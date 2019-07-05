#include <planar_bot_planning/prm.h>
#include <ros/ros.h>
#include <planar_bot.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

void writeCSV(std::ofstream& file, const std::vector<planar_bot_planning::Node>& data)
{
  for(size_t i=0; i<data.size(); i++) {
    for(size_t j=0; j<data[i].size(); j++) {
      file << std::to_string(data[i][j]) << ",";
    }
    file << "\n";
  }
}

void writeCSV(std::ofstream& file, const std::vector<planar_bot_planning::Edge>& data)
{
  for(size_t i=0; i<data.size(); i++) {
    for(size_t j=0; j<data[i].size(); j++) {
      file << std::to_string(data[i][j]) << ",";
    }
    file << "\n";
  }
}

int main(int argc, char **argv) 
{  
  if(argc < 4) {
    ROS_ERROR("Usage: ./create_prm_node <number of milestones> <filename_nodes> <filename_edges>");
    return 0;
  }
  
  int N = std::stoi(argv[1]);
  std::string filename_nodes = argv[2];
  std::string filename_edges = argv[3];
  
  planar_bot::Robot robot;
  
  std::array<double,4> link_length = {0.9,0.9,0.9,0.95};
  std::array<double,4> link_radius = {0.07,0.07,0.07,0.07};
  robot.setLinkGeometry(link_length, link_radius);
  
  std::array<double,4> q_min   = {-0.15, -2.8, -2.8, -2.8};
  std::array<double,4> q_max   = {  3.3,  2.8,  2.8,  2.8};
  std::array<double,4> dq_max  = {0.5, 0.5, 0.5, 0.5};
  std::array<double,4> ddq_max = {0.1, 0.1, 0.1, 0.1};
  robot.setJointLimits(q_min, q_max, dq_max, ddq_max);
  
  std::vector<am_ssv_dist::PSS_object> pss_obstacles(2);
  std::vector<am_ssv_dist::LSS_object> lss_obstacles(1);
  
  pss_obstacles[0].set_p0(Eigen::Vector4f(1.0, 0.6, 0.0, 0.0));
  pss_obstacles[0].set_radius(0.1);
  pss_obstacles[1].set_p0(Eigen::Vector4f(2.0, 0.6, 0.0, 0.0));
  pss_obstacles[1].set_radius(0.1);
  
  lss_obstacles[0].set_p0(Eigen::Vector4f(-3.0, -0.4, 0.0, 0.0));
  lss_obstacles[0].set_p1(Eigen::Vector4f( 3.0, -0.4, 0.0, 0.0));
  lss_obstacles[0].set_radius(0.05);
  
  planar_bot_planning::PRM_Param param;
  // a ratio of 5 edges per node is good for 2 dof -> ratio ~ 2^d for equal connectivity
  param.num_nodes = N;
  param.minimum_sample_distance = 0.2;
  param.maximum_connect_distance = 0.35;
  
  planar_bot_planning::PRM planner;
  
  planner.createRoadmap(&robot, pss_obstacles, lss_obstacles, param);
  
  std::vector<planar_bot_planning::Node> nodes;
  std::vector<planar_bot_planning::Edge> edges;
  planner.getRoadmap(nodes, edges);
  
  std::ofstream file;
  
  file.open(filename_nodes);
  writeCSV(file, nodes);
  file.close();
  
  file.open(filename_edges);
  writeCSV(file, edges);
  file.close();
  
  return 0;
}
