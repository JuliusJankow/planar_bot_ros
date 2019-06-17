#ifndef PIK_CONTROLLER_H
#define PIK_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/JointControllerState.h>
#include <array>
#include <ssv/ssv_objects.h>
#include <kinematics/kinematics.h>
#include <planning/spline.h>

namespace planar_bot_control
{

class PIKController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
public:
  PIKController();
  ~PIKController();

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);  
  void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< hardware_interface::JointHandle > joints_;

private:
  ros::Subscriber sub_task_space_goal_;

  Eigen::Vector4d getOptimizedNullSpaceCommand(const Eigen::Vector2d& dx, const double& sample_time);
  Eigen::Vector4d getGradientJLA(const Robot& robot);
  Eigen::Vector4d computeLinkObstacleGradient(const Robot& robot, const int link,
     const am_ssv_dist::LSS_object* lss, const am_ssv_dist::PSS_object* pss);
  Eigen::Vector4d computeLinkPairGradient(const Robot& robot,
                                          const int link_p0, const am_ssv_dist::LSS_object* lss0,
                                          const int link_p1, const am_ssv_dist::LSS_object* lss1);
  Eigen::Vector4d getGradientCA(const Robot& robot, const std::array<am_ssv_dist::LSS_object, 4>& link_ssv);
  Eigen::Vector4d getGradientSCA(const Robot& robot, const std::array<am_ssv_dist::LSS_object, 4>& link_ssv);
  Eigen::Vector4d getGradientOfCost(const Robot& robot);

  Eigen::Matrix2d K_e_;
  
  Robot robot_;
  
  Spline spline_;
  
  Eigen::Vector4d u_;
  
  ros::Time last_update_time_;

  void goalCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  
  std::vector<double> hard_limits_min_;
  std::vector<double> hard_limits_max_;
  std::vector<double> soft_limits_min_;
  std::vector<double> soft_limits_max_;
  
  // parameter for preceding horizon
  int num_opt_iter_{20};
  int t_horizon_{5};
  double k_q_{100};
  double k_dq_{10};
  double k_du_{0.1};
  double alpha_opt_{0.8};
  
  // parameter for local optimization
  double d_sca_{0.2};
  double m_sca_{50.0};
  double d_obs_{0.2};
  double m_obs_{50.0};
  double k_jla_{2.0};
  
  // obstacle parameter
  double x1_{1.0},y1_{0.6},radius1_{0.1};
  double x2_{2.0},y2_{0.6},radius2_{0.1};
}; // class

} // namespace

#endif
