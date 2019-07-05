#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>

#include "trajectory_controller.h"

namespace planar_bot_control {

SimpleController::SimpleController() {}
SimpleController::~SimpleController() {sub_trajectory_.shutdown();}

bool SimpleController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
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

  sub_trajectory_ = n.subscribe<std_msgs::Float64MultiArray>("trajectory", 4, &SimpleController::trajectoryCB, this);
  
  return true;
}

void SimpleController::update(const ros::Time& t, const ros::Duration& period)
{
  if(counter_ < trajectory_.rows()) {
    for(int i=0; i<joints_.size(); i++) {
      joints_[i].setCommand(trajectory_(counter_,i));
    }
    counter_++; 
  }
}

void SimpleController::trajectoryCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  
  int N = msg->layout.dim[0].size;
  int D = msg->layout.dim[1].size;

  if(N <= 0 || D != 4) {
    ROS_ERROR("Dimension not correct");
    return;
  }
  
  if(counter_ < trajectory_.rows()) return;
  
  std::vector<double> data = msg->data;
  Eigen::Map<Eigen::MatrixXd> mat(data.data(), N, D);
  trajectory_ = mat;
    
  counter_ = 0;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(planar_bot_control::SimpleController, controller_interface::ControllerBase)
