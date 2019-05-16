#ifndef RMRC_CONTROLLER_H
#define RMRC_CONTROLLER_H

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

namespace planar_bot_control
{

struct Spline {
    std::array<double, 6> coeff{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    double start_time{0.0};
    double end_time{0.0};
};

class RMRCController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    RMRCController();
    ~RMRCController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);
    void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<Spline> > spline_buffer_;

private:
    ros::Subscriber sub_task_space_goal_;

    bool sampleTaskSpline(const double& t, Eigen::Vector2d& w_d, Eigen::Vector2d& dw_d);
    void getTaskSpaceParam(const Eigen::Vector4d& q, Eigen::Vector2d& w, Eigen::Matrix<double,2,4>& J);
    void constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end, const double t_end);
    void constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end);

    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;
    std::vector<double> link_length_;
    Eigen::Vector4d q_d_;
    Eigen::Matrix2d K_e_;
    double k_p_{400.0};
    double k_d_{100.0};
    double dw_max_{0.2};

    void goalCB(const std_msgs::Float64MultiArrayConstPtr& msg);
    // void enforceJointLimits(double &command, unsigned int index);
}; // class

} // namespace

#endif
