#pragma once

#include <vector>
#include <array>
#include <Eigen/Dense>
#include <ssv/ssv_objects.h>

#include <planar_bot.h>

namespace planar_bot_planning
{

using Path = std::vector< Eigen::VectorXd >;

class ElasticBand
{

public:

  struct Param {
    double stride{0.1};
    double spring_constant{0.0};
    double relative_rest_length{1.0};
    double lambda{0.05};
    double d_active{0.1};
    size_t num_iterations{10000};
  };

  ElasticBand(planar_bot::Robot* robot) : robot_(robot) {}

  bool smooth(Path& result, const Path& initial_path, const Param& param, 
              const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
              const std::vector<am_ssv_dist::LSS_object>& lss_obstacles);
              
  void preparePath(Path& path, const Param& param);

private:

  double update(Path& path, const Param& param, 
                const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                const std::vector<am_ssv_dist::LSS_object>& lss_obstacles);
              
  void getObstacleForce(Eigen::VectorXd& F_obs, const Eigen::VectorXd& q, const Param& param,
                        const std::vector<am_ssv_dist::PSS_object>& pss_obstacles, 
                        const std::vector<am_ssv_dist::LSS_object>& lss_obstacles);

  planar_bot::Robot* robot_;

};

} // namespace
