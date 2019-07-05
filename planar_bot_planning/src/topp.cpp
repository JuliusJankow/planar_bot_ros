#include <planar_bot_planning/topp.h>
#include <math.h>
#include <iostream>

using namespace planar_bot_planning;

double TOPP::solve(Trajectory& trajectory, const Path& path)
{
  size_t N = path.size();

  Path v_path(N);
  std::vector<double> T(N),ds(N);
  T[0] = 0.0;
  ds[0] = 0.0;
  ds[N-1] = 0.0;
  v_path[N-1] = Eigen::VectorXd::Zero(path[0].size());
  
  // backward integration
  for(size_t n=N-2; n > 0; n--) {
    Eigen::VectorXd q_pm = (path[n+1] - path[n-1]) / 2;
    Eigen::VectorXd q_pn = (path[n+1] - path[n]);
    
    // find maximal path velocity and maximal path deceleration
    double ds_max, dds_min;
    ds_max = robot_->dq_max_[0] / std::abs(q_pm[0]);
    dds_min = - robot_->ddq_max_[0] / std::abs(q_pn[0]);
    for(size_t d=1; d<q_pm.size(); d++) {
      ds_max = std::min( ds_max, robot_->dq_max_[d] / std::abs(q_pm[d]) );
      dds_min = std::max( dds_min, - robot_->ddq_max_[d] / std::abs(q_pn[d]));
    }
    // compute maximal possible path velocity inside dynamic limits
    ds[n] = std::min( ds_max, std::sqrt(ds[n+1]*ds[n+1] - 2*dds_min) );
  }
  
  // forward integration
  for(size_t n=1; n<N-1; n++) {
    Eigen::VectorXd q_pm = (path[n+1] - path[n-1]) / 2;
    Eigen::VectorXd q_nm = (path[n]   - path[n-1]);
    
    // find maximal path acceleration for segment
    double dds_max = robot_->ddq_max_[0] / std::abs(q_nm[0]);
    for(size_t d=1; d<q_pm.size(); d++) {
      dds_max = std::min( dds_max, robot_->ddq_max_[d] / std::abs(q_nm[d]));
    }
    // compute maximal admissible path velocity inside dynamic limits
    ds[n] = std::min( ds[n], std::sqrt(ds[n-1]*ds[n-1] + 2*dds_max) );
    v_path[n] = ds[n] * q_pm;
    T[n] = T[n-1] + 2/(ds[n]+ds[n-1]);
  }
  T[N-1] = T[N-2] + 2/ds[N-2];
  
  // resampling with trajectory samples equally spaced in time
  size_t K = std::floor(T[N-1] / trajectory.sample_time);
  
  trajectory.q.resize(K+1);
  trajectory.dq.resize(K+1);
  
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
    
    trajectory.q[k] = path[n_] + (path[n_+1] - path[n_]) * (t - T[n_]) * 
      (ds[n_] + 0.5*(ds[n_+1] - ds[n_]) * (t - T[n_]) / (T[n_+1] - T[n_]));
      
    trajectory.dq[k] = v_path[n_] + (v_path[n_+1] - v_path[n_]) * (t - T[n_]) / (T[n_+1] - T[n_]);
  }
  
  trajectory.q[K] = path[N-1];
  trajectory.dq[K] = Eigen::VectorXd::Zero(trajectory.q[K].size());
  
  return T[N-1];
}
