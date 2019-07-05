#pragma once

#include <vector>
#include <Eigen/Dense>
#include <planar_bot.h>

namespace planar_bot_planning
{

using Path = std::vector< Eigen::VectorXd >;

struct Trajectory {
  std::vector< Eigen::VectorXd > q;
  std::vector< Eigen::VectorXd > dq;
  double sample_time{1.0};
};

class TOPP
{

public:

  TOPP(planar_bot::Robot* robot) : robot_(robot) {}

  double solve(Trajectory& trajectory, const Path& path);

private:

  planar_bot::Robot* robot_;

};

} // namespace
