#ifndef COMMON_VAR
#define COMMON_VAR

namespace planar_bot_control {

// joint limits
static constexpr double q0_min = -0.3, q0_max = 3.45;
static constexpr double q0_min_soft = -0.15, q0_max_soft = 3.3;
static constexpr double q_abs = 3.0, q_abs_soft = 2.8;

// parameter (self) collision avoidance (sca)
static constexpr double kD_sca = 0.2;
static constexpr double kM_sca = 400.0;

// obstacle parameter
static constexpr double kX1 = 1.0, kY1 = 0.6, kRadius1 = 0.1;
static constexpr double kX2 = 2.0, kY2 = 0.6, kRadius2 = 0.1;

} // namespace

#endif
