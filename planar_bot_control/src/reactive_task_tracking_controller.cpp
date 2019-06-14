#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>

#include "reactive_task_tracking_controller.h"
#include "common_global_var.h"
#include "ssv/ssv_dist_calc.h"

namespace planar_bot_control {

RTTController::RTTController() {}
RTTController::~RTTController() {sub_task_space_goal_.shutdown();}

inline void generatePowers(int n, const double& x, std::vector<double>& powers) {
    powers[0] = 1.0;
    for (int i=1; i<=n; ++i) {
      powers[i] = powers[i-1]*x;
    }
}


bool RTTController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {
  // List of controlled joints
  std::string param_name = "joints";
  if(!n.getParam(param_name, joint_names_)) {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }

  if(joint_names_.size() == 0) {
    ROS_ERROR_STREAM("List of joint names is empty.");
    return false;
  }
  
  double k_p = 0.0;
  if(!n.getParam("task_space_p_gain", k_p)) {
    ROS_WARN_STREAM("Failed to getParam task_space_error_feedback_gain, using default: " << k_p);
  }
  K_p_ << k_p, 0.0, 0.0, k_p;
  
  double k_d = 2.0*std::sqrt(k_p);
  K_d_ << k_d, 0.0, 0.0, k_d;

  // Get URDF
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  for(unsigned int i=0; i<joint_names_.size(); i++)
  {
    try
    {
      joints_.push_back(hw->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }

    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
    if (!joint_urdf)
    {
      ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
      return false;
    }
    joint_urdfs_.push_back(joint_urdf);

    link_length_.push_back(0.9);
    
    link_ssv_[i].set_radius(0.07);
  }
  link_length_[3] = 0.95;
  
  std::vector<Spline> splines(2);
  spline_buffer_.writeFromNonRT(splines);
  
  robot_.w[0] = Eigen::Vector2d::Zero();
  robot_.J[0] = Eigen::Matrix<double,2,4>::Zero();

  sub_task_space_goal_ = n.subscribe<std_msgs::Float64MultiArray>("goal", 1, &RTTController::goalCB, this);
  return true;
}

void RTTController::starting(const ros::Time& time) {
  // initialize desired joint positions
  robot_.q[0] = 1.7;
  robot_.q[1] = -0.5;
  robot_.q[2] = -0.5;
  robot_.q[3] = 0.3;
  
  updateRobotState();
  x_des = robot_.w[4];
  dx_des << 0.0, 0.0;
}

void RTTController::constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end, const double t_end) {
    std::vector<Spline> splines;

    double lambda = 1/(t_end- t_start);
    std::vector<double> lambda_pow(6);
    generatePowers(5,lambda,lambda_pow);

    for(int i=0; i<2; i++) {
      Spline spline;

      spline.start_time = t_start;
      spline.end_time = t_end;

      spline.coeff[0] = w_start(i);
      spline.coeff[1] = 0.0;
      spline.coeff[2] = 0.0;
      spline.coeff[3] = 10.0 * (w_end(i) - w_start(i)) * lambda_pow[3];
      spline.coeff[4] = -15.0 * (w_end(i) - w_start(i)) * lambda_pow[4];
      spline.coeff[5] = 6.0 * (w_end(i) - w_start(i)) * lambda_pow[5];

      splines.push_back(spline);
    }

    spline_buffer_.writeFromNonRT(splines);
}

void RTTController::constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end) {
    // TODO: spline parametrized by max velocity
}

bool RTTController::sampleTaskSpline(const double& t, Eigen::Vector2d& w_d, Eigen::Vector2d& dw_d) {
    std::vector<Spline>* splines = spline_buffer_.readFromRT();

    if((*splines)[0].start_time > t || t > (*splines)[0].end_time) {
        return false;
    }

    double t_rel = t - (*splines)[0].start_time;

    std::vector<double> t_pow(6);
    generatePowers(5,t_rel,t_pow);

    w_d(0) = (*splines)[0].coeff[0];
    w_d(1) = (*splines)[1].coeff[0];
    dw_d << 0.0, 0.0;

    for(int i=1; i<6; i++) {
      w_d(0)  += (*splines)[0].coeff[i] * t_pow[i];
      dw_d(0) += (*splines)[0].coeff[i] * t_pow[i-1] * (double)i;
      w_d(1)  += (*splines)[1].coeff[i] * t_pow[i];
      dw_d(1) += (*splines)[1].coeff[i] * t_pow[i-1] * (double)i;
    }

    return true;
}

Eigen::Vector4d RTTController::getTauFromSubtasks()
{
  // transform robot ssv models
  for(int link=0; link<4; link++) {
    link_ssv_[link].set_p0(Eigen::Vector4f(robot_.w[link](0), robot_.w[link](1), 0.0, 0.0));
    link_ssv_[link].set_p1(Eigen::Vector4f(robot_.w[link+1](0), robot_.w[link+1](1), 0.0, 0.0));
  }
  
  // create obstacle ssv models
  am_ssv_dist::PSS_object p1,p2;
  p1.set_p0(Eigen::Vector4f(kX1, kY1, 0.0, 0.0));
  p1.set_radius(kRadius1);
  p2.set_p0(Eigen::Vector4f(kX2, kY2, 0.0, 0.0));
  p2.set_radius(kRadius2);
  
  Eigen::Vector4d tau_sub(Eigen::Vector4d::Zero());
  
  // compute joint limit avoidance (JLA) torques
  tau_sub += getSubtaskTorqueJLA(robot_);
  
  // compute collision avoidance (CA) torques
  for(int link=0; link<4; link++) {
    tau_sub += getSubtaskTorqueCA(robot_, link, &(link_ssv_[link]), &p1);
    tau_sub += getSubtaskTorqueCA(robot_, link, &(link_ssv_[link]), &p2);
  }
  
  // compute self collision avoidance (SCA) torques
  tau_sub += getSubtaskTorqueSCA(robot_, 0, 2, &(link_ssv_[0]), &(link_ssv_[2]));
  tau_sub += getSubtaskTorqueSCA(robot_, 0, 3, &(link_ssv_[0]), &(link_ssv_[3]));
  tau_sub += getSubtaskTorqueSCA(robot_, 1, 3, &(link_ssv_[1]), &(link_ssv_[3]));
  
  return tau_sub;
}

void RTTController::updateRobotState() {
  std::vector<double> lc,ls;
  for(int i=0; i<4; i++) {
    double link_angle = 0.0;
    for(int j=0; j<=i; j++) {
      link_angle += robot_.q[j];
    }
    lc.emplace_back(link_length_[i] * std::cos(link_angle));
    ls.emplace_back(link_length_[i] * std::sin(link_angle));
  }
  
  for(int frame=1; frame<=4; frame++) { // calculate position and jacobian at [frame]
    for(int joint=0; joint<frame; joint++) { // calculate partial derivative for [joint]
      robot_.J[frame](0,joint) = 0.0;
      robot_.J[frame](1,joint) = 0.0;
      for(int i=(frame-1); i>=joint; i--) {
        robot_.J[frame](0,joint) -= ls[i];
        robot_.J[frame](1,joint) += lc[i];
      }
    }
    robot_.w[frame](0) =  robot_.J[frame](1,0);
    robot_.w[frame](1) = -robot_.J[frame](0,0);
  }
}

void RTTController::update(const ros::Time& t, const ros::Duration& period) {
  for(unsigned int i=0; i<joint_names_.size(); i++) {
    robot_.q[i]  = joints_[i].getPosition();
    robot_.dq[i] = joints_[i].getVelocity();
  }

  // update forward kinematics and jacobian
  updateRobotState();

  Eigen::Vector2d x(robot_.w[4]);
  Eigen::MatrixXd J_EE(robot_.J[4]);
  // compute moore-penrose pseudo inverse
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(J_EE);
  Eigen::MatrixXd J_inv = cod.pseudoInverse();
  // compute Null Space projector
  Eigen::Matrix4d N = Eigen::Matrix4d::Identity() - J_inv * J_EE;

  Eigen::Vector2d w_d, dw_d, F_task;
  if(sampleTaskSpline(trajectory_time_, w_d, dw_d)) {
    Eigen::Vector2d e_x = x - w_d;
    
    double min_dist = 0.1, max_dist = 0.3;
    double time_scale = (max_dist - e_x.norm()) / (max_dist - min_dist);
    time_scale = std::max(0.0, std::min(1.0, time_scale));
    
    x_des = w_d;
    dx_des = time_scale * dw_d;
    
    trajectory_time_ += time_scale * period.toSec();
  }
  Eigen::Vector2d dx(J_EE*robot_.dq);
  
  // compute task space control law
  F_task = - K_p_ * (x - x_des) - K_d_ * (dx - dx_des);

  // compute torque to locally optimize subtasks
  Eigen::Vector4d tau_pot = getTauFromSubtasks();

  Eigen::Vector4d tau_c = J_EE.transpose() * F_task + tau_pot - 5.0 * robot_.dq;

  double tau_max = 80.0;
  double multiplicator = 1.0;
  for(unsigned int i=0; i<joint_names_.size(); i++) {
    double tau_abs = std::abs(tau_c[i]);
    if(tau_abs > tau_max) multiplicator = std::min(multiplicator, tau_max/tau_abs);
  }
  for(unsigned int i=0; i<joint_names_.size(); i++) {
    joints_[i].setCommand(tau_c[i]*multiplicator);
  }
}

void RTTController::goalCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 2) return;
  
  Eigen::Vector2d w_end(msg->data[0], msg->data[1]);
  
  double t_start = 0.0;
  
  constructSpline(robot_.w[4], t_start, w_end, t_start + 3.0);
  
  trajectory_time_ = t_start;
}

} // namespace

PLUGINLIB_EXPORT_CLASS(planar_bot_control::RTTController, controller_interface::ControllerBase)
