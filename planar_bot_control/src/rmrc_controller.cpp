#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>
#include <Eigen/Dense>
#include <Eigen/QR>

#include "rmrc_controller.h"

namespace planar_bot_control {

static constexpr double kSampleTime = 0.001; // in seconds

RMRCController::RMRCController() {}
RMRCController::~RMRCController() {sub_task_space_goal_.shutdown();}

inline void generatePowers(int n, const double& x, std::vector<double>& powers) {
    powers[0] = 1.0;
    for (int i=1; i<=n; ++i) {
      powers[i] = powers[i-1]*x;
    }
}

bool RMRCController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) {
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
  
  double k_e = 0.0;
  if(!n.getParam("task_space_error_feedback_gain", k_e)) {
    ROS_WARN_STREAM("Failed to getParam task_space_error_feedback_gain, using default: " << k_e);
  }
  K_e_.diagonal() << k_e, k_e;

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
  }
  link_length_[3] = 0.95;
  
  std::vector<Spline> splines(2);
  spline_buffer_.writeFromNonRT(splines);

  sub_task_space_goal_ = n.subscribe<std_msgs::Float64MultiArray>("goal", 1, &RMRCController::goalCB, this);
  return true;
}

void RMRCController::starting(const ros::Time& time) {
    // initialize desired joint positions
    for(unsigned int i=0; i<joint_names_.size(); i++) {
        q_d_[i]  = joints_[i].getPosition();
    }
}

void RMRCController::constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end, const double t_end) {
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

void RMRCController::constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end) {
    // TODO: spline parametrized by max velocity
}

bool RMRCController::sampleTaskSpline(const double& t, Eigen::Vector2d& w_d, Eigen::Vector2d& dw_d) {
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

void RMRCController::getTaskSpaceParam(const Eigen::Vector4d& q, Eigen::Vector2d& w, Eigen::Matrix<double,2,4>& J) {
  std::vector<double> lc,ls;
  for(int i=0; i<4; i++) {
    double link_angle = 0.0;
    for(int j=0; j<=i; j++) {
      link_angle += q[j];
    }
    lc.emplace_back(link_length_[i] * std::cos(link_angle));
    ls.emplace_back(link_length_[i] * std::sin(link_angle));
  }

  for(int i=0; i<4; i++) {
    J(0,i) = 0.0;
    J(1,i) = 0.0;
    for(int j=3; j>=i; j--) {
     J(0,i) -= ls[j];
     J(1,i) += lc[j];
    }
  }

  w(0) =  J(1,0);
  w(1) = -J(0,0);
}

void RMRCController::update(const ros::Time& t, const ros::Duration& period) {

    Eigen::Vector2d w, w_d, dw_d;
    Eigen::Vector4d dq_d(0.0, 0.0, 0.0, 0.0);

    if(sampleTaskSpline(t.toSec(), w_d, dw_d)) {
        Eigen::Matrix<double,2,4> J;

        getTaskSpaceParam(q_d_, w, J);

        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(J);
        Eigen::MatrixXd J_inv = cod.pseudoInverse();


        dq_d = J_inv * (K_e_ * (w_d - w) + dw_d);
    }

    q_d_ += dq_d * period.toSec(); // kSampleTime;

    double tau_max = 80.0;
    for(unsigned int i=0; i<joint_names_.size(); i++) {
        double q  = joints_[i].getPosition();
        double dq = joints_[i].getVelocity();
        double tau_c = k_p_ * (q_d_(i) - q) + k_d_ * (dq_d(i) - dq);
        tau_c = std::min(std::max(tau_c, -tau_max), tau_max);
        joints_[i].setCommand(tau_c);
    }
}

void RMRCController::goalCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 2) return;
  
  Eigen::Vector2d w_start, w_end(msg->data[0], msg->data[1]);
  Eigen::Matrix<double,2,4> J;
  
  getTaskSpaceParam(q_d_, w_start, J);
  
  double t_start = ros::Time::now().toSec() + 2.0;
  
  constructSpline(w_start, t_start, w_end, t_start + 3.0);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(planar_bot_control::RMRCController, controller_interface::ControllerBase)
