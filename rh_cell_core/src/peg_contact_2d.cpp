#include "rh_cell_core/peg_contact_2d.hpp"

#include <algorithm>
#include <cmath>

namespace rh_cell_core
{
Eigen::Vector2d PegPortContact::force(const Eigen::Vector2d& p)
{
  const double side = (p.y() >= 0.0) ? 1.0 : -1.0;  // offset side: that corner can touch
  const Eigen::Vector2d pm(p.x(), std::abs(p.y()));  // mirrored to the y >= 0 side
  const double yc = pm.y() + g_.half_width;          // corner distance from the port axis
  const double x = pm.x();
  const double outer = g_.half_open + g_.chamfer;

  int surf = 0;
  double pen = 0.0;
  Eigen::Vector2d n(0.0, 0.0), t(0.0, 0.0);  // normal = force direction on the peg; tangent
  if (x > 0.0 && yc >= outer && x < g_.thickness)
  {
    surf = 1;  // flat wall face outside the chamfer
    pen = x;
    n = {-1.0, 0.0};
    t = {0.0, 1.0};
  }
  else if (x > 0.0 && x <= g_.chamfer)
  {
    pen = (yc + x - outer) / std::sqrt(2.0);  // chamfer line |y| + x = outer
    if (pen > 0.0)
    {
      surf = 2;
      n = Eigen::Vector2d(-1.0, -1.0) / std::sqrt(2.0);
      t = Eigen::Vector2d(1.0, -1.0) / std::sqrt(2.0);
    }
  }
  else if (x > g_.chamfer && x <= g_.thickness + 0.2)
  {
    pen = yc - g_.half_open;  // port side wall
    if (pen > 0.0)
    {
      surf = 3;
      n = {0.0, -1.0};
      t = {1.0, 0.0};
    }
  }

  const Eigen::Vector2d dp = has_prev_ ? Eigen::Vector2d(pm - prev_) : Eigen::Vector2d::Zero();
  prev_ = pm;
  has_prev_ = true;
  if (surf == 0 || pen <= 0.0)
  {
    surf_ = 0;
    z_ = 0.0;
    return Eigen::Vector2d::Zero();
  }
  if (surf != surf_)
    z_ = 0.0;  // new contact: the stiction spring starts relaxed
  surf_ = surf;

  const double fn = g_.k_env * pen;
  z_ += dp.dot(t);
  const double limit = g_.mu * fn / g_.k_t;
  z_ = std::clamp(z_, -limit, limit);  // beyond the friction cone the contact slips
  const Eigen::Vector2d fm = fn * n - g_.k_t * z_ * t;
  return {fm.x(), side * fm.y()};
}
}  // namespace rh_cell_core
