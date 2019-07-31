#ifndef RTT_CONTROLLER_H
#define RTT_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <array>
#include <kinematics/kinematics.h>
#include <planning/spline.h>
#include <ssv/ssv_objects.h>

namespace planar_bot_control
{

class RTTController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  RTTController();
  ~RTTController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< hardware_interface::JointHandle > joints_;

private:
  ros::Subscriber sub_task_space_goal_;
  
  Eigen::Vector4d getTauFromSubtasks(std::vector<double>& da);
  Eigen::Vector4d getSubtaskTorqueJLA();
  Eigen::Vector4d getSubtaskTorqueCA(const int link_idx, 
                                     const am_ssv_dist::LSS_object* lss, const am_ssv_dist::PSS_object* pss, double& da);
  Eigen::Vector4d getSubtaskTorqueSCA(const int link_idx_0, const int link_idx_1,
                                      const am_ssv_dist::LSS_object* lss_0, const am_ssv_dist::LSS_object* lss_1);

  Eigen::Matrix2d K_p_;
  Eigen::Matrix2d K_d_;
  
  // parameter for local optimization
  double d_sca_{0.15};
  double m_sca_{3.0};
  double damping_sca_{5.0};
  double d_obs_{0.15};
  double m_obs_{3.0};
  double k_jla_{2.0};
  double d_jla_{10.0};
  double inflation_factor_{0.8};
  double minimum_active_distance_{0.05};
  
  // obstacle parameter
  double x1_{1.0},y1_{0.6},radius1_{0.1};
  double x2_{2.0},y2_{0.6},radius2_{0.1};
  
  double joint_damping_{5.0};
  
  Robot robot_;
  
  Spline spline_;
  
  std::vector<double> hard_limits_min_;
  std::vector<double> hard_limits_max_;
  std::vector<double> soft_limits_min_;
  std::vector<double> soft_limits_max_;
  
  Eigen::Vector2d x_des, dx_des;
  
  double time_scaling_lower_thresh_{0.2};
  double time_scaling_upper_thresh_{0.5};
  double trajectory_time_;
  
  void goalCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  
  realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>* x_des_pub;
  realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>* d_active_pub;
}; // class

} // namespace

#endif
