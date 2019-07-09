#include <planar_bot_planning/topp.h>
#include <cmath>
#include <iostream>

using namespace planar_bot_planning;

inline void generatePowers(int n, const double& x, std::vector<double>& powers) {
  powers[0] = 1.0;
  for (int i=1; i<=n; ++i) {
    powers[i] = powers[i-1]*x;
  }
}

inline double getCubicSpline(const std::array<double,3>& q, const double df_0, CubicSpline& spline)
{ // compute spline for segment q[0] -> q[1]
  spline[0] = q[0];
  spline[1] = df_0;
  spline[3] = 1.5 * q[0] - 2 * q[1] + 0.5 * q[2] + df_0;
  spline[2] = - spline[3] + q[1] - q[0] - df_0;
  return (0.5 * (q[2] - q[0])); // return df_1
}

inline double getStartSpline(const std::array<double,3>& q, CubicSpline& spline)
{ // compute quadratic spline for segment q[0] -> q[1]
  spline[0] = q[0];
  spline[1] = -1.5 * q[0] + 2 * q[1] - 0.5 * q[2];
  spline[2] = 0.5 * q[0] - q[1] + 0.5 * q[2];
  spline[3] = 0.0;
  return (0.5 * (q[2] - q[0])); // return df_1
}

inline void getEndSpline(const std::array<double,2>& q, const double df_0, CubicSpline& spline)
{ // compute quadratic spline for segment q[0] -> q[1]
  spline[0] = q[0];
  spline[1] = df_0;
  spline[2] = - q[0] + q[1] - df_0;
  spline[3] = 0.0;
}

Eigen::VectorXd TOPP::interpolateSegment(const Path& segment, const Eigen::VectorXd& df_0, CubicSegment& cubic_segment)
{
  Eigen::VectorXd df_1(robot_->num_dof_);
  cubic_segment.clear();
  CubicSpline spline;
  std::array<double,3> q;
  for(size_t i=0; i<robot_->num_dof_; i++) {
    q[0] = segment[0](i);
    q[1] = segment[1](i);
    q[2] = segment[2](i);
    df_1(i) = getCubicSpline(q, df_0(i), spline);
    cubic_segment.push_back(spline);
  }
  return df_1;
}

Eigen::VectorXd TOPP::interpolateStartSegment(const Path& segment, CubicSegment& cubic_segment)
{
  Eigen::VectorXd df_1(robot_->num_dof_);
  cubic_segment.clear();
  CubicSpline spline;
  std::array<double,3> q;
  for(size_t i=0; i<robot_->num_dof_; i++) {
    q[0] = segment[0](i);
    q[1] = segment[1](i);
    q[2] = segment[2](i);
    df_1(i) = getStartSpline(q, spline);
    cubic_segment.push_back(spline);
  }
  return df_1;
}

void TOPP::interpolateEndSegment(const Path& segment, const Eigen::VectorXd& df_0, CubicSegment& cubic_segment)
{
  cubic_segment.clear();
  CubicSpline spline;
  std::array<double,2> q;
  for(size_t i=0; i<robot_->num_dof_; i++) {
    q[0] = segment[0](i);
    q[1] = segment[1](i);
    getEndSpline(q, df_0(i), spline);
    cubic_segment.push_back(spline);
  }
}

void TOPP::interpolatePath(const Path& path, CubicPath& cubic_path)
{
  cubic_path.clear();
  Path segment(3);
  segment[0] = path[0];
  segment[1] = path[1];
  segment[2] = path[2];
  CubicSegment cubic_segment;
  Eigen::VectorXd df_0 = interpolateStartSegment(segment, cubic_segment);
  cubic_path.push_back(cubic_segment);
  for(size_t k=1; k<path.size()-2; k++) {
    segment[0] = path[k];
    segment[1] = path[k+1];
    segment[2] = path[k+2];
    df_0 = interpolateSegment(segment, df_0, cubic_segment);
    cubic_path.push_back(cubic_segment);
  }
  Path end_segment(2);
  end_segment[0] = path[path.size()-2];
  end_segment[1] = path[path.size()-1];
  interpolateEndSegment(end_segment, df_0, cubic_segment);
  cubic_path.push_back(cubic_segment);
}

inline double f(const CubicSpline& spline, const double s)
{
  double result = 0.0;
  std::vector<double> s_pow(4);
  generatePowers(3, s, s_pow);
  for(size_t n=0; n<4; n++) {
    result += spline[n] * s_pow[n];
  }
  return result;
}

inline double df(const CubicSpline& spline, const double s)
{
  double result = 0.0;
  std::vector<double> s_pow(3);
  generatePowers(2, s, s_pow);
  for(size_t n=0; n<3; n++) {
    result += (double)(n+1) * spline[n+1] * s_pow[n];
  }
  return result;
}

inline double ddf(const CubicSpline& spline, const double s)
{
  return 0.0; //(2.0*spline[2] + 6.0*spline[3]*s);
}

void TOPP::sampleCubicPath(const CubicPath& cubic_path, const double _s, 
  Eigen::VectorXd& q, Eigen::VectorXd& dq_ds, Eigen::VectorXd& ddq_ds)
{
  size_t k = std::floor(_s);
  if(k < 0 || k >= cubic_path.size()) {
    std::cerr << "s is out of range. k = " << k << std::endl;
    return;
  }
  double s = _s - (double)k;
  CubicSegment segment = cubic_path[k];
  
  for(size_t d=0; d<robot_->num_dof_; d++) {
    q(d) = f(segment[d], s);
    dq_ds(d) = df(segment[d], s);
    ddq_ds(d) = ddf(segment[d], s);
  }
}

double TOPP::solve(Trajectory& trajectory, const Path& path_in)
{
  size_t N = path_in.size();
  
  Path path = path_in;
  /*std::vector< CircularBlend > blends;
  computeCircularBlends(path, blends, 0.02);*/
  
  CubicPath cubic_path;
  interpolatePath(path_in, cubic_path);

  std::vector<double> T(N),ds(N);
  T[0] = 0.0;
  ds[0] = 0.0;
  ds[N-1] = 0.0;
  Eigen::VectorXd q(robot_->num_dof_), dq_ds(robot_->num_dof_), ddq_ds(robot_->num_dof_);
  double s = N-1;
  sampleCubicPath(cubic_path, s, q, dq_ds, ddq_ds);
  
  // backward integration
  for(size_t n=N-2; n > 0; n--) {
    double ds_max, dds_min;
    // compute dds_min for point n+1
    dds_min = - robot_->ddq_max_[0] - ddq_ds(0)*ds[n+1]*ds[n+1] / std::abs(dq_ds(0));
    for(size_t d=1; d<robot_->num_dof_; d++) {
      dds_min = std::max( dds_min, - robot_->ddq_max_[d] - ddq_ds(d)*ds[n+1]*ds[n+1] / std::abs(dq_ds(d)) );
    }
    // sample path and derivatives for s
    s = n;
    sampleCubicPath(cubic_path, s, q, dq_ds, ddq_ds);
    // find maximal path velocity and maximal path deceleration
    ds_max = robot_->dq_max_[0] / std::abs(dq_ds(0));
    dds_min = std::max( dds_min, - robot_->ddq_max_[0] - ddq_ds(0)*ds[n+1]*ds[n+1] / std::abs(dq_ds(0) - 2*ddq_ds(0)) );
    for(size_t d=1; d<robot_->num_dof_; d++) {
      ds_max = std::min(ds_max, robot_->dq_max_[d] / std::abs(dq_ds(d)));
      dds_min = std::max( dds_min, - robot_->ddq_max_[d] - ddq_ds(d)*ds[n+1]*ds[n+1] / std::abs(dq_ds(d) - 2*ddq_ds(d)) );
    }
    // compute maximal possible path velocity inside dynamic limits
    ds[n] = std::min( ds_max, std::sqrt(ds[n+1]*ds[n+1] - 2*dds_min) );
  }
  
  s = 0;
  sampleCubicPath(cubic_path, s, q, dq_ds, ddq_ds);
  // forward integration
  for(size_t n=1; n<N-1; n++) {
    // compute dds_max for point n-1
    double dds_max = robot_->ddq_max_[0] - ddq_ds(0)*ds[n-1]*ds[n-1] / std::abs(dq_ds(0));
    for(size_t d=1; d<robot_->num_dof_; d++) {
      dds_max = std::min( dds_max, robot_->ddq_max_[d] - ddq_ds(d)*ds[n-1]*ds[n-1] / std::abs(dq_ds(d)) );
    }
    s = n;
    sampleCubicPath(cubic_path, s, q, dq_ds, ddq_ds);
    // find maximal path acceleration for segment
    dds_max = std::min( dds_max, robot_->ddq_max_[0] - ddq_ds(0)*ds[n-1]*ds[n-1] / std::abs(dq_ds(0) + 2*ddq_ds(0)) );
    for(size_t d=1; d<robot_->num_dof_; d++) {
      dds_max = std::min( dds_max, robot_->ddq_max_[d] - ddq_ds(d)*ds[n-1]*ds[n-1] / std::abs(dq_ds(d) + 2*ddq_ds(d)) );
    }
    // compute maximal admissible path velocity inside dynamic limits
    ds[n] = std::min( ds[n], std::sqrt(ds[n-1]*ds[n-1] + 2*dds_max) );
    T[n] = T[n-1] + 2/(ds[n]+ds[n-1]);
  }
  T[N-1] = T[N-2] + 2/ds[N-2];
  
  // resampling with trajectory samples equally spaced in time
  size_t K = std::floor(T[N-1] / trajectory.sample_time);
  
  trajectory.q.resize(K);
  trajectory.dq.resize(K);
  
  for(size_t k=0; k<K; k++) {
    double t = k * trajectory.sample_time;
    
    // find corresponding path segment
    size_t n_;
    for(size_t n=0; n<N-1; n++) {
      if(T[n+1] > t) {
        n_ = n;
        break;
      }
    }
    
    double s = n_ + (t - T[n_]) * (ds[n_] + 0.5*(ds[n_+1] - ds[n_]) * (t - T[n_]) / (T[n_+1] - T[n_]));
    double ds_dt = ds[n_] + (ds[n_+1] - ds[n_]) * (t - T[n_]) / (T[n_+1] - T[n_]);
    sampleCubicPath(cubic_path, s, q, dq_ds, ddq_ds);
    trajectory.q[k] = q;
    trajectory.dq[k] = dq_ds * ds_dt;
  }
  
  //trajectory.q[K] = path[N-1];
  //trajectory.dq[K] = Eigen::VectorXd::Zero(trajectory.q[K].size());
  
  return T[N-1];
}
