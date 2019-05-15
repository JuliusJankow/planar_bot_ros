#include <Eigen3.h>

namespace planar_bot_control {

void constructSpline() {
  
}

void sampleTaskSpline(const ros::Time& t, Eigen::Vector2d& w_d, Eigen::Vector2d& dw_d) {
  std::vector<double> t_pow = computePower(t,5);
  w_d  = spline_coeff_[0];
  dw_d << 0.0, 0.0;
  
  for(int i=1; i<6; i++) {
    w_d  += spline_coeff_[i] * t_pow[i];
    dw_d += spline_coeff_[i] * t_pow[i-1] * (double)i;
  }
}

void getTaskSpaceParam(const Eigen3::Vector4d& q, Eigen::Vector2d& w, Eigen::Matrix<> J) {
  std::vector<double> lc,ls;
  for(int i=0; i<4; i++) {
    double link_angle = 0.0;
    for(int j=0; j<=i; j++) {
      link_angle += q[j];
    }
    lc.emplace_back(l[i] * std::cos(link_angle);
    ls.emplace_back(l[i] * std::sin(link_angle);
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

void update(const ros::Time& t) {
  Eigen::Vector4d q(x,x,x,x);
  Eigen::Vector4d dq(x,x,x,x);
  
  getTaskSpaceParam(q_d, w, J);
  
  computeWheightedPseudoInverse(J, W, J_inv);
  
  sampleTaskSpline(t, w_d, dw_d);
  
  Eigen::Vector4d dq_d = J_inv * (K_e * (w_d - w) + dw_d);
  
  q_d += dq_d * kSampleTime;
  
  Eigen::Vector4d tau = K_p * (q_d - q) + K_d * (dq_d - dq);
  
  setCommand(tau);
}

} // namespace
