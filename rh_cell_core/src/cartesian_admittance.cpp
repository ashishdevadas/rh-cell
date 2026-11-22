#include "rh_cell_core/cartesian_admittance.hpp"

#include <algorithm>

namespace rh_cell_core
{
CartesianAdmittance::CartesianAdmittance(const CartesianAdmittanceParams& p, const Eigen::Vector3d& x0)
  : p_(p), x_(x0)
{
}

void CartesianAdmittance::reset(const Eigen::Vector3d& x0)
{
  x_ = x0;
  v_.setZero();
}

Eigen::Vector3d CartesianAdmittance::step(const Eigen::Vector3d& x_des, const Eigen::Vector3d& f_ext, double dt)
{
  for (int i = 0; i < 3; ++i)
  {
    const double a = (f_ext[i] - p_.damping[i] * v_[i] - p_.stiffness[i] * (x_[i] - x_des[i])) / p_.mass[i];
    v_[i] += a * dt;  // semi-implicit Euler, as Admittance1D
    x_[i] += v_[i] * dt;
    const double lim = p_.max_deflection[i];
    const double dev = x_[i] - x_des[i];
    if (dev > lim || dev < -lim)
    {
      x_[i] = x_des[i] + std::clamp(dev, -lim, lim);
      v_[i] = 0.0;  // stop at the limit instead of bouncing
    }
  }
  return x_;
}
}  // namespace rh_cell_core
