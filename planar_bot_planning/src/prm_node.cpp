#include <planar_bot_planning/prm.h>
#include <planar_bot_planning/elastic_band.h>
#include <planar_bot_planning/topp.h>
#include <ros/ros.h>
#include <planar_bot.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

Eigen::Vector4d q_init;
planar_bot_planning::Trajectory trajectory;
planar_bot_planning::PRM* planner;
planar_bot::Robot robot;
std::vector<am_ssv_dist::PSS_object> pss_obstacles(2);
std::vector<am_ssv_dist::LSS_object> lss_obstacles(1);

void readCSV(std::istream& input, std::vector<planar_bot_planning::Node>& output)
{
  std::string csv_line;
  output.clear();
  
  while( std::getline(input, csv_line) ) {
    istringstream csv_stream(csv_line);
    std::vector<double> csv_row;
    std::string csv_element;
    
    while( getline(csv_stream, csv_element, ',') ) {
      double value = std::stod(csv_element);
      csv_row.push_back(value);
    }
    
    planar_bot_planning::Node node(csv_row.size());
    for(size_t i=0; i<csv_row.size(); i++) {
      node[i] = csv_row[i];
    }
    
    output.push_back(node);
  }
}
void readCSV(std::istream& input, std::vector<planar_bot_planning::Edge>& output)
{
  std::string csv_line;
  output.clear();
  
  while( std::getline(input, csv_line) ) {
    istringstream csv_stream(csv_line);
    std::string csv_element;
    
    planar_bot_planning::Edge edge;
    getline(csv_stream, csv_element, ',');
    edge[0] = std::stoi(csv_element);
    getline(csv_stream, csv_element, ',');
    edge[1] = std::stoi(csv_element);
    
    output.push_back(edge);
  }
}

void qCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 4) return;
  
  q_init = Eigen::Vector4d(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}

void goalCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 2) return;
  
  Eigen::Vector2d x_goal(msg->data[0], msg->data[1]);
    
  planar_bot_planning::Path path_feasible, path_smooth;
  
  ROS_INFO("Start planning ...");
  
  planner->searchPath(path_feasible, q_init, x_goal, 0.08);
  planner->shortcutPath(path_feasible, pss_obstacles, lss_obstacles);
  
  ROS_INFO_STREAM("Search done. " << path_feasible.size() << " via points constructed; \nq_end = " << path_feasible.back().transpose());
  ROS_INFO("Start smoothing ...");
  
  planar_bot_planning::ElasticBand eb(&robot);
  planar_bot_planning::ElasticBand::Param param_eb;
  param_eb.stride = 0.01;
  param_eb.spring_constant = 5.0;
  param_eb.relative_rest_length = 0.8;
  param_eb.lambda = 0.1;
  param_eb.num_iterations = 10000;
  param_eb.d_active = 0.1;
  
  eb.preparePath(path_feasible, param_eb);
  
  if(!eb.smooth(path_smooth, path_feasible, param_eb, pss_obstacles, lss_obstacles)) {
    ROS_ERROR("Smoothing did not work. Elastic Band snapped.");
    return;
  }
  
  ROS_INFO("Smoothing done.");
  ROS_INFO("Start time parametrization ...");
  
  //param_eb.stride = 0.05;
  //eb.preparePath(path_smooth, param_eb);
  planar_bot_planning::TOPP topp(&robot);
  planar_bot_planning::Trajectory trajectory_tmp;
  trajectory_tmp.sample_time = 0.02;
  double t_end = topp.solve(trajectory_tmp, path_smooth);
  
  ROS_INFO_STREAM("Time parametrization done. Motion will take " << t_end << " seconds.");
  
  trajectory = trajectory_tmp;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planar_bot_prm");
  ros::NodeHandle n;
  
  if(argc < 3) {
    ROS_ERROR("Please specify a filename for nodes and edges!");
    return 0;
  }
  
  // read already constructed graph
  
  std::string filename_nodes = argv[1];
  std::string filename_edges = argv[2];
  
  std::vector<planar_bot_planning::Node> nodes;
  {
    std::fstream file(filename_nodes, ios::in);
    if(!file.is_open()) {
      ROS_ERROR("File not found");
      return 0;
    }
    readCSV(file, nodes);
  }
  
  std::vector<planar_bot_planning::Edge> edges;
  {
    std::fstream file(filename_edges, ios::in);
    if(!file.is_open()) {
      ROS_ERROR("File not found");
      return 0;
    }
    readCSV(file, edges);
  }
  
  // define robot geometry + robot joint limits; TODO: Read from URDF
  
  std::array<double,4> link_length = {0.9,0.9,0.9,0.95};
  std::array<double,4> link_radius = {0.07,0.07,0.07,0.07};
  robot.setLinkGeometry(link_length, link_radius);
  
  std::array<double,4> q_min   = {-0.15, -2.8, -2.8, -2.8};
  std::array<double,4> q_max   = {  3.3,  2.8,  2.8,  2.8};
  std::array<double,4> dq_max  = {0.5, 0.5, 0.5, 0.5};
  //std::array<double,4> ddq_max = {0.1, 0.1, 0.1, 0.1};
  std::array<double,4> ddq_max = {20.0, 40.0, 80.0, 100.0};
  robot.setJointLimits(q_min, q_max, dq_max, ddq_max);
  
  planner = new planar_bot_planning::PRM(&robot, nodes, edges);
  
  // define obstacles in workspace by using SSVs
  
  pss_obstacles[0].set_p0(Eigen::Vector4f(1.0, 0.6, 0.0, 0.0));
  pss_obstacles[0].set_radius(0.1);
  pss_obstacles[1].set_p0(Eigen::Vector4f(2.0, 0.6, 0.0, 0.0));
  pss_obstacles[1].set_radius(0.1);
  
  lss_obstacles[0].set_p0(Eigen::Vector4f(-3.0, -0.4, 0.0, 0.0));
  lss_obstacles[0].set_p1(Eigen::Vector4f( 3.0, -0.4, 0.0, 0.0));
  lss_obstacles[0].set_radius(0.05);
  
  q_init = Eigen::Vector4d(1.7,-0.5,-0.5,0.3);
  
  ros::Rate rate(1.0);
  
  ros::Subscriber start_sub = n.subscribe<std_msgs::Float64MultiArray>("q", 4, &qCB);
  ros::Subscriber goal_sub = n.subscribe<std_msgs::Float64MultiArray>("x", 4, &goalCB);
  ros::Publisher array_pub = n.advertise<std_msgs::Float64MultiArray>("/planar_bot/traj_controller/trajectory", 4);
  
  std_msgs::Float64MultiArray array;
  array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  array.layout.dim.push_back(std_msgs::MultiArrayDimension());
  array.layout.dim[0].label = "Via Points";
  array.layout.dim[1].label = "DoF";
  
  while(ros::ok()) {
    if(trajectory.q.size() > 0) {
      array.layout.dim[0].size = trajectory.q.size();
      array.layout.dim[1].size = 4;
      
      array.data.clear();
      for(size_t dof=0; dof<4; dof++) {
        for(size_t k=0; k<trajectory.q.size(); k++) {
          array.data.push_back(trajectory.q[k](dof));
        }
      }
      
      array_pub.publish(array);
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
