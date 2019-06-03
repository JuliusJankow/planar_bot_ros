#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Dense>

#include "predictive_ik_controller.h"
#include "ssv/ssv_dist_calc.h"

namespace planar_bot_control {

static constexpr double kSampleTime = 0.001; // in seconds

// parameter (self) collision avoidance (sca)
static constexpr double kD_sca{0.15};
static constexpr double kM_sca{3.0};

// parameter for preceding horizon
static constexpr int kNumOptIter{20};
static constexpr int kTHorizon{150};
static constexpr double kKQ{1.0};
static constexpr double kKDq{1.0};
static constexpr double kKDu{1.0};
static constexpr double kAlphaPH{0.5};

PIKController::PIKController() {}
PIKController::~PIKController() {sub_task_space_goal_.shutdown();}

inline void generatePowers(int n, const double& x, std::vector<double>& powers) {
    powers[0] = 1.0;
    for (int i=1; i<=n; ++i) {
      powers[i] = powers[i-1]*x;
    }
}

bool PIKController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n) {
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
    
    robot_.link_ssv[i].set_radius(0.07);
  }
  link_length_[3] = 0.95;
  
  std::vector<Spline> splines(2);
  spline_buffer_.writeFromNonRT(splines);
  
  robot_.w[0] = Eigen::Vector2d::Zero();
  robot_.J[0] = Eigen::Matrix<double,2,4>::Zero();

  sub_task_space_goal_ = n.subscribe<std_msgs::Float64MultiArray>("goal", 1, &PIKController::goalCB, this);
  return true;
}

void PIKController::starting(const ros::Time& time) {
    // initialize desired joint positions
    for(unsigned int i=0; i<joint_names_.size(); i++) {
        robot_.q[i]  = joints_[i].getPosition();
        u_[i] = 0.0;
    }
}

void PIKController::constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end, const double t_end) {
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

void PIKController::constructSpline(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end) {
    // TODO: spline parametrized by max velocity
}

bool PIKController::sampleTaskSpline(const double& t, Eigen::Vector2d& w_d, Eigen::Vector2d& dw_d) {
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

double q0_min = -0.3, q0_min_s = -0.15;
double q0_max =  3.45, q0_max_s =  3.3;
double q_m =  3.0, q_m_s =  2.8;
Eigen::Vector4d getGradientJLA(const RobotState& robot) {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);
  
  if(robot.q(0) < q0_min_s) {
    gradient(0) = 2 * (robot.q(0) - q0_min_s) / ((q0_min - q0_min_s) * (q0_min - q0_min_s));
  }
  if(robot.q(0) > q0_max_s) {
    gradient(0) = 2 * (robot.q(0) - q0_max_s) / ((q0_max - q0_max_s) * (q0_max - q0_max_s));
  }
  for(int i=1; i<4; i++) {
    if(robot.q(i) < -q_m_s) {
      gradient(i) = 2 * (robot.q(i) + q_m_s) / ((q_m - q_m_s) * (q_m - q_m_s));
    }
    if(robot.q(i) > q_m_s) {
      gradient(i) = 2 * (robot.q(i) - q_m_s) / ((q_m - q_m_s) * (q_m - q_m_s));
    }
  }
  
  return gradient;
}

Eigen::Vector4d computeLinkObstacleGradient(const RobotState& robot, int link, am_ssv_dist::PSS_object* p) {
  Eigen::Vector4d gradient = Eigen::Vector4d::Zero();

  if(0 > link || link > 3) {
    return gradient;
  }

  // compute shortest distance and corresponding link points
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  dist_calc.cdPL(p,&(robot.link_ssv[link]),dist_result);
  
  if(dist_result.distance >= kD_sca) {
    return gradient;
  }
  
  // compute point jacobians
  Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Vector2d p1_l = p1 - robot.w[link];
  Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link; column++) {
    J_p1.col(column) << -p1_l(1), p1_l(0);
  }
  J_p1 += robot.J[link];
  
  // compute gradient
  gradient = kM_sca * (dist_result.distance - kD_sca) * J_p1.transpose() * (p1 - p0) / dist_result.distance;
  
  return gradient;
}

Eigen::Vector4d computeLinkPairGradient(const RobotState& robot, int link_p0, int link_p1) {
  Eigen::Vector4d gradient = Eigen::Vector4d::Zero();

  if(0 > link_p0 || link_p0 > 3 || 0 > link_p1 || link_p1 > 3) {
    return gradient;
  }

  // compute shortest distance and corresponding link points
  am_ssv_dist::SSV_DistCalculator dist_calc;
  am_ssv_dist::SSV_DistCalcResult dist_result;
  dist_calc.cdLL(&(robot.link_ssv[link_p0]),&(robot.link_ssv[link_p1]),dist_result);
  
  if(dist_result.distance >= kD_sca) {
    return gradient;
  }
  
  // compute point jacobians
  Eigen::Vector2d p0(dist_result.start_point(0), dist_result.start_point(1));
  Eigen::Vector2d p0_l = p0 - robot.w[link_p0];
  Eigen::Matrix<double,2,4> J_p0 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link_p0; column++) {
    J_p0.col(column) << -p0_l(1), p0_l(0);
  }
  J_p0 += robot.J[link_p0];
  
  Eigen::Vector2d p1(dist_result.end_point(0), dist_result.end_point(1));
  Eigen::Vector2d p1_l = p1 - robot.w[link_p1];
  Eigen::Matrix<double,2,4> J_p1 = Eigen::Matrix<double,2,4>::Zero();
  for(int column=0; column<=link_p1; column++) {
    J_p1.col(column) << -p1_l(1), p1_l(0);
  }
  J_p1 += robot.J[link_p1];
  
  // compute gradient
  gradient = kM_sca * (dist_result.distance - kD_sca) * (J_p0.transpose() - J_p1.transpose()) * (p0 - p1) / dist_result.distance;
  
  return gradient;
}

double x1 = 1.0, y1 = 0.5, radius1 = 0.1;
double x2 = 2.0, y2 = 0.5, radius2 = 0.1;
Eigen::Vector4d getGradientCA(const RobotState& robot) {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);
  
  am_ssv_dist::PSS_object p1,p2;
  p1.set_p0(Eigen::Vector4f(x1, y1, 0.0, 0.0));
  p1.set_radius(radius1);
  p2.set_p0(Eigen::Vector4f(x2, y2, 0.0, 0.0));
  p2.set_radius(radius2);
  
  for(int link=0; link<4; link++) {
    gradient += computeLinkObstacleGradient(robot, link, &p1);
    gradient += computeLinkObstacleGradient(robot, link, &p2);
  }
  
  return gradient;
}

Eigen::Vector4d getGradientSCA(const RobotState& robot) {
  Eigen::Vector4d gradient(0.0, 0.0, 0.0, 0.0);
  
  gradient += computeLinkPairGradient(robot, 0,2);
  gradient += computeLinkPairGradient(robot, 0,3);
  gradient += computeLinkPairGradient(robot, 1,3);
  
  return gradient;
}

Eigen::Vector4d getGradientOfCost(RobotState& robot) {
  // transform ssv models
  for(int link=0; link<4; link++) {
    robot.link_ssv[link].set_p0(Eigen::Vector4f(robot.w[link](0), robot.w[link](1), 0.0, 0.0));
    robot.link_ssv[link].set_p1(Eigen::Vector4f(robot.w[link+1](0), robot.w[link+1](1), 0.0, 0.0));
  }
  
  return getGradientJLA(robot) + getGradientSCA(robot) + getGradientCA(robot);
}

Eigen::Matrix4d calculateHessFq(const RobotState& robot, const Eigen::Matrix<double,2,4>& J_inv, 
                                const Eigen::Vector2d& dx, const Eigen::Vector4d& u)
{
  /*Eigen::Vector4d G1(Eigen::Vector4d::Zero()), G2(Eigen::Vector4d::Zero());
  for(int j=0; j<4; j++) { // for each joint j
    for(int i=j; i<4; i++) { // for each link i actuated by joint j
      double link_angle = 0.0;
      for(int n=0; n<i; n++) { // for each joint n influencing position of link i
        link_angle += robot.q[n];
      }
      G1(j) -= link_length_[i] * cos(link_angle);
      G2(j) -= link_length_[i] * sin(link_angle);
    }
  }*/
  
  Eigen:Matrix4d JinvTimesJq(Eigen::Matrix4d::Zero());
  for(int k=0; k<4; k++) { // for each joint k as second derivative
    Eigen::Matrix<double,2,4> Jqk;
    for(int j=0; j<4; j++) { // for each joint j as first derivative
      int idx = std::max(j,k);
      Jqk(0,j) = -J(1,idx); // G1(idx);
      Jqk(1,j) =  J(0,idx); // G2(idx);
    }
    JinvTimesJq += J_inv * Jqk;
  }
  
  
}

Eigen::Vector4d PIKController::getOptimizedNullSpaceCommand(const Eigen::Matrix4d& N0, 
                                                            const Eigen::Vector4d& dq_task)
{
  RobotState predicted_robot;
  for(int i=0; i<4; i++) {
    predicted_robot.link_ssv[i].set_radius(0.07);
  }
  
  Eigen::Vector4d dq, u, du(0.0, 0.0, 0.0, 0.0), lambda1, dlambda1, lambda2, dlambda2;
  
  for(int i=0; i<kNumOptIter; i++) {
    predicted_robot.q = robot_.q;
    u = u_;
    for(int k=0; k<kTHorizon; k++) {
      dq = dq_task + N0 * u;
      predicted_robot.q = predicted_robot.q + kSampleTime * dq;
      u = u + kSampleTime * du;
    }
    
    updateRobotState(predicted_robot);
    Eigen::MatrixXd J(predicted_robot.J[4]);

    // compute moore-penrose pseudo inverse
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(J);
    Eigen::MatrixXd J_inv = cod.pseudoInverse();
    // compute Null Space projector
    Eigen::Matrix4d N1 = Eigen::Matrix4d::Identity() - J_inv * J;
    
    Eigen::Matrix4d hess_fq(calculateHessFq(predicted_robot, J_inv));
    
    Eigen::Vector4d grad_H(getGradientOfCost(predicted_robot));
    
    lambda1 = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
    lambda2 = lambda1;
    for(int k=kTHorizon; k>0; k--) {
      dlambda1 = - hess_fq * (lambda1 + kKDq * dq) - kKQ * grad_H;
      dlambda2 = - N1 * (lambda1 + kKDq * dq);
      lambda1 = lambda1 - kSampleTime * dlambda1;
      lambda2 = lambda2 - kSampleTime * dlambda2;
    }
    
    du = (1 - kAlphaPH * kKDu) * du - kAlphaPH * lambda2;
  }
  
  return du;
}

void PIKController::updateRobotState(RobotState& robot) {
// efficient forward kinematics calculation for planar bot
  std::vector<double> lc,ls;
  for(int i=0; i<4; i++) {
    double link_angle = 0.0;
    for(int j=0; j<=i; j++) {
      link_angle += robot.q[j];
    }
    lc.emplace_back(link_length_[i] * std::cos(link_angle));
    ls.emplace_back(link_length_[i] * std::sin(link_angle));
  }
  
  for(int frame=1; frame<=4; frame++) { // calculate position and jacobian at [frame]
    for(int joint=0; joint<frame; joint++) { // calculate partial derivative for [joint]
      robot.J[frame](0,joint) = 0.0;
      robot.J[frame](1,joint) = 0.0;
      for(int i=(frame-1); i>=joint; i--) {
        robot.J[frame](0,joint) -= ls[i];
        robot.J[frame](1,joint) += lc[i];
      }
    }
    robot.w[frame](0) =  robot.J[frame](1,0);
    robot.w[frame](1) = -robot.J[frame](0,0);
  }
}

void PIKController::update(const ros::Time& t, const ros::Duration& period) {
    Eigen::Vector2d w, w_d, dw_d;
    Eigen::Vector4d dq_task(0.0, 0.0, 0.0, 0.0);

    // update forward kinematics and jacobian
    updateRobotState(robot_);
    Eigen::MatrixXd J(robot_.J[4]);

    // compute moore-penrose pseudo inverse
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(J);
    Eigen::MatrixXd J_inv = cod.pseudoInverse();
    // compute Null Space projector
    Eigen::Matrix4d N = Eigen::Matrix4d::Identity() - J_inv * J;

    if(sampleTaskSpline(t.toSec(), w_d, dw_d)) {
        // compute desired joint velocity for task
        dq_task = J_inv * (K_e_ * (w_d - robot_.w[4]) + dw_d);
    }

    // optimize null space acceleration for a fixed horizon
    u_ = u_ + kSampleTime * getOptimizedNullSpaceCommand(N, dq_task);
    
    Eigen::Vector4d dq_d = dq_task + alpha_ * N * u_;
    robot_.q += dq_d * kSampleTime;

    // double tau_max = 80.0;
    for(unsigned int i=0; i<joint_names_.size(); i++) {
        /*double q  = joints_[i].getPosition();
        double dq = joints_[i].getVelocity();
        double tau_c = k_p_ * (virtual_robot_.q(i) - q) + k_d_ * (dq_d(i) - dq);
        tau_c = std::min(std::max(tau_c, -tau_max), tau_max);
        joints_[i].setCommand(tau_c);*/
        joints_[i].setCommand(robot_.q[i]);
    }
}

void PIKController::goalCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if(msg->data.size() != 2) return;
  
  Eigen::Vector2d w_end(msg->data[0], msg->data[1]);
  
  double t_start = ros::Time::now().toSec() + 1.0;
  
  constructSpline(robot_.w[4], t_start, w_end, t_start + 3.0);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(planar_bot_control::PIKController, controller_interface::ControllerBase)
