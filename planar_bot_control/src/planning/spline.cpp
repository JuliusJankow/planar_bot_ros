#include "planning/spline.h"

inline void generatePowers(int n, const double& x, std::vector<double>& powers) {
  powers[0] = 1.0;
  for (int i=1; i<=n; ++i) {
    powers[i] = powers[i-1]*x;
  }
}

Spline::Spline()
{
  std::vector<Quintic> splines;
  Quintic q;
  for(int i=0; i<6; i++) {
    q[i] = 0.0;
  }
  splines.push_back(q);
  splines.push_back(q);
  
  rt_coefficient_buffer_.writeFromNonRT(splines);
}

void Spline::construct(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end, const double t_end)
{
  std::vector<Quintic> splines = *(rt_coefficient_buffer_.readFromNonRT());
  
  start_time = t_start;
  end_time = t_end;

  double lambda = 1/(t_end- t_start);
  std::vector<double> lambda_pow(6);
  generatePowers(5,lambda,lambda_pow);

  for(int i=0; i<2; i++) {
    splines[i][0] = w_start(i);
    splines[i][1] = 0.0;
    splines[i][2] = 0.0;
    splines[i][3] = 10.0 * (w_end(i) - w_start(i)) * lambda_pow[3];
    splines[i][4] = -15.0 * (w_end(i) - w_start(i)) * lambda_pow[4];
    splines[i][5] = 6.0 * (w_end(i) - w_start(i)) * lambda_pow[5];
  }

  rt_coefficient_buffer_.writeFromNonRT(splines);
}

void Spline::construct(const Eigen::Vector2d& w_start, const double t_start, const Eigen::Vector2d& w_end)
{
  // TODO: spline parametrized by max velocity
}

bool Spline::sample(const double& t, Eigen::Vector2d& w_d, Eigen::Vector2d& dw_d)
{
  std::vector<Quintic>* splines = rt_coefficient_buffer_.readFromNonRT();

  if(start_time > t || t > end_time) {
    return false;
  }

  double t_rel = t - start_time;

  std::vector<double> t_pow(6);
  generatePowers(5,t_rel,t_pow);

  for(int i=0; i<2; i++) {
    w_d(i) = (*splines)[i][0];
    dw_d(i) = 0.0;

    for(int j=1; j<6; j++) {
      w_d(i)  += (*splines)[i][j] * t_pow[j];
      dw_d(i) += (*splines)[i][j] * t_pow[j-1] * (double)j;
    }
  }

  return true;
}
