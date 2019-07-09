#pragma once

#include <vector>
#include <Eigen/Dense>
#include <planar_bot.h>

namespace planar_bot_planning
{

using Path = std::vector< Eigen::VectorXd >;

using CubicSpline = std::array<double, 4>; // f(s) = a0 + a1 * s + a2 * s^2 + a3 * s^3
using CubicSegment = std::vector<CubicSpline>; // a spline for each dof
using CubicPath = std::vector<CubicSegment>; // a Segment between every pair of way points

struct Trajectory {
  std::vector< Eigen::VectorXd > q;
  std::vector< Eigen::VectorXd > dq;
  double sample_time{1.0};
};

class TOPP
{

public:

  TOPP(planar_bot::Robot* robot) : robot_(robot) {}
  
  void interpolatePath(const Path& path, CubicPath& cubic_path);

  void sampleCubicPath(const CubicPath& cubic_path, const double _s, 
    Eigen::VectorXd& q, Eigen::VectorXd& dq_ds, Eigen::VectorXd& ddq_ds);

  double solve(Trajectory& trajectory, const Path& path_in);

private:
  
  Eigen::VectorXd interpolateSegment(const Path& segment, const Eigen::VectorXd& df_0, CubicSegment& cubic_segment);
  Eigen::VectorXd interpolateStartSegment(const Path& segment, CubicSegment& cubic_segment);
  void interpolateEndSegment(const Path& segment, const Eigen::VectorXd& df_0, CubicSegment& cubic_segment);

  planar_bot::Robot* robot_;

};

} // namespace
