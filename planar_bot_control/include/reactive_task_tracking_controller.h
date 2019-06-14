#ifndef RTT_CONTROLLER_H
#define RTT_CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <array>

#include "ssv/ssv_objects.h"

namespace planar_bot_control
{

struct Spline {
  std::array<double, 6> coeff{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double start_time{0.0};
  double end_time{0.0};
};

struct RobotState {
  Eigen::Vector4d q;
  Eigen::Vector4d dq;
  std::array<Eigen::Vector2d, 5> w;
  std::array<Eigen::Matrix<double,2,4> , 5> J;
};

class RTTController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  RTTController();
  ~RTTController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<Spline> > spline_buffer_;

private:
  ros::Subscriber sub_task_space_goal_;

  void updateRobotState();
  
  bool sampleTaskSpline(const double& t, Eigen::Vector2d& w_d, Eigen::Vector2d& dw_d);
  void constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end, const double t_end);
  void constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end);
  
  Eigen::Vector4d getTauFromSubtasks();

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
  std::vector<double> link_length_;
  
  Eigen::Matrix2d K_p_;
  Eigen::Matrix2d K_d_;
  
  double dw_max_{0.2};
  double alpha_{1.0};
  
  // parameter (self) collision avoidance (sca)
  double d_sca_{0.15};
  double m_sca_{3.0};
  
  RobotState robot_;
  
  Eigen::Vector2d x_des, dx_des;
  
  double trajectory_time_;
  
  std::array<am_ssv_dist::LSS_object, 4> link_ssv_;

  void goalCB(const std_msgs::Float64MultiArrayConstPtr& msg);
}; // class

Eigen::Vector4d getSubtaskTorqueJLA(const RobotState& robot);
Eigen::Vector4d getSubtaskTorqueCA(const RobotState& robot, const int link_idx, 
                                          const am_ssv_dist::LSS_object* lss, const am_ssv_dist::PSS_object* pss);
Eigen::Vector4d getSubtaskTorqueSCA(const RobotState& robot, const int link_idx_0, const int link_idx_1,
                                           const am_ssv_dist::LSS_object* lss_0, const am_ssv_dist::LSS_object* lss_1);

} // namespace

#endif
