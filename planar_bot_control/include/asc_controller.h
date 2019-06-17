#ifndef RTT_CONTROLLER_H
#define RTT_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <array>
#include <kinematics/kinematics.h>
#include <planning/spline.h>
#include <ssv/ssv_objects.h>

namespace planar_bot_control
{

class ASCController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  ASCController();
  ~ASCController();

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< hardware_interface::JointHandle > joints_;

private:
  ros::Subscriber sub_task_space_goal_;

  Eigen::Vector4d getGradientJLA();
  Eigen::Vector4d getGradientSCA(std::array<am_ssv_dist::LSS_object, 4>& link_ssv);
  Eigen::Vector4d computeLinkPairGradient(int link_p0, am_ssv_dist::LSS_object* lss0,
                                          int link_p1, am_ssv_dist::LSS_object* lss1);
  Eigen::Vector4d getGradientCA(std::array<am_ssv_dist::LSS_object, 4>& link_ssv);
  Eigen::Vector4d computeLinkObstacleGradient(int link, am_ssv_dist::LSS_object* lss, am_ssv_dist::PSS_object* pss);
  Eigen::Vector4d getGradientOfCost();

  Eigen::Matrix2d K_e_;
  
  // parameter for local optimization
  double d_sca_{0.15};
  double m_sca_{3.0};
  double d_obs_{0.15};
  double m_obs_{3.0};
  double k_jla_{2.0};
  
  // obstacle parameter
  double x1_{1.0},y1_{0.6},radius1_{0.1};
  double x2_{2.0},y2_{0.6},radius2_{0.1};
  
  Robot robot_;
  
  Spline spline_;
  
  std::vector<double> hard_limits_min_;
  std::vector<double> hard_limits_max_;
  std::vector<double> soft_limits_min_;
  std::vector<double> soft_limits_max_;

  void goalCB(const std_msgs::Float64MultiArrayConstPtr& msg);
}; // class

} // namespace

#endif
