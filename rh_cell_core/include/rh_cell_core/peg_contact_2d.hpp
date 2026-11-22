#pragma once
// Planar contact model of a square cassette entering a chamfered port (RH Cell 2.0, step 2).
// Used by the unit tests and by chamfer_demo to check the controller idea WITHOUT Gazebo.
//
// Coordinates: x = insertion depth of the cassette front face, measured from the wall front face
// (x < 0: still in front of the wall); y = lateral position of the cassette centre relative to
// the TRUE port centre. Only the cassette corner on the side of the offset can touch.
//
//   wall front face        x = 0, for |y_corner| > half_open + chamfer
//   chamfer (45 deg)       0 < x < chamfer: half opening = half_open + chamfer - x
//   port side wall         chamfer < x < thickness: half opening = half_open
//
// Normal force: penalty spring (k_env). Friction: stick-slip (elasto-plastic) model - a
// tangential spring that holds while |f_t| < mu * f_n and slips beyond. Unlike a smoothed
// Coulomb law it really sticks, so wedging (mu > tan(chamfer angle)) is reproduced instead of
// a slow creep.

#include <Eigen/Core>

namespace rh_cell_core
{
struct PegPortGeometry
{
  double half_width = 0.035;  // cassette half width [m]
  double half_open = 0.037;   // port half opening behind the chamfer [m]
  double chamfer = 0.005;     // 45 deg chamfer depth [m]
  double thickness = 0.020;   // wall thickness [m]
  double k_env = 5.0e4;       // normal contact stiffness [N/m]
  double k_t = 5.0e4;         // tangential (stiction) stiffness [N/m]
  double mu = 0.3;            // friction coefficient
};

class PegPortContact
{
public:
  explicit PegPortContact(const PegPortGeometry& g) : g_(g) {}
  // p = (x, y) cassette front-face position. Returns the force on the cassette (x, y).
  Eigen::Vector2d force(const Eigen::Vector2d& p);
  // 0 none, 1 wall face, 2 chamfer, 3 port side wall
  int surface() const { return surf_; }

private:
  PegPortGeometry g_;
  int surf_ = 0;
  double z_ = 0.0;  // tangential spring deflection while in contact with one surface
  Eigen::Vector2d prev_{0.0, 0.0};
  bool has_prev_ = false;
};
}  // namespace rh_cell_core
