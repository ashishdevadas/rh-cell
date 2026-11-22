#pragma once
// Guidance virtual fixture for teleoperated insertion (Phase 1.3).
//
// The port defines an axis: a point `origin` (the port entrance on the robot side) and a unit
// direction `dir` (the insertion direction). Inside the "capture zone" the operator's velocity
// command v is reshaped:
//
//   v_axial = (v . dir) dir                 kept: the operator decides how fast to go in/out
//   v_perp  = v - v_axial                   scaled by k_perp (0 = locked on the axis, 1 = free)
//   v_pull  = k_pull * (closest point on axis - p)   pulls the tool back onto the axis
//
// Forward motion (into the port) is blocked while the tool is more than `entry_tolerance` off the
// axis, so the cassette cannot be driven into the wall. Outside the capture zone the command
// passes through unchanged.
// No ROS here on purpose (unit-testable).

#include <array>

namespace rh_cell_core
{
using Vec3 = std::array<double, 3>;

struct FixtureParams
{
  Vec3 origin{0.0, 0.0, 0.0};
  Vec3 dir{1.0, 0.0, 0.0};          // normalised in the constructor
  double capture_radius = 0.05;     // zone radius around the axis [m]
  double capture_behind = 0.15;     // zone starts this far before the origin, along -dir [m]
  double k_perp = 0.1;              // fraction of sideways command that passes
  double k_pull = 2.0;              // pull-back gain [1/s]
  double entry_tolerance = 0.004;   // max off-axis error allowed to move forward [m]
};

struct FixtureOutput
{
  Vec3 v{0.0, 0.0, 0.0};
  bool engaged = false;          // inside the capture zone
  bool forward_blocked = false;  // operator tried to go in while off-axis
  double axial = 0.0;            // position along the axis from origin [m]
  double radial = 0.0;           // distance from the axis [m]
};

class VirtualFixture
{
public:
  explicit VirtualFixture(const FixtureParams& p);
  FixtureOutput apply(const Vec3& p, const Vec3& v_cmd) const;

private:
  FixtureParams p_;
};
}  // namespace rh_cell_core
