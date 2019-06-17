#ifndef SPLINE_H
#define SPLINE_H

#include <Eigen/Dense>
#include <realtime_tools/realtime_buffer.h>
#include <array>

class Spline
{
using Quintic = std::array<double, 6>;

public:
  Spline();
  ~Spline(){}

  void construct(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end, const double t_end);
  void construct(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end);
  bool sample(const double& t, Eigen::Vector2d& w_d, Eigen::Vector2d& dw_d);
private:
  realtime_tools::RealtimeBuffer< std::vector<Quintic> > rt_coefficient_buffer_;
  double start_time{0.0};
  double end_time{0.0};
};

#endif
