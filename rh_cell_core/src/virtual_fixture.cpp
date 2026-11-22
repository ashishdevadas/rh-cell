#include "rh_cell_core/virtual_fixture.hpp"

#include <cmath>

namespace rh_cell_core
{
namespace
{
double dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vec3 add(const Vec3& a, const Vec3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
Vec3 scale(const Vec3& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
}  // namespace

VirtualFixture::VirtualFixture(const FixtureParams& p) : p_(p)
{
  const double n = norm(p_.dir);
  p_.dir = scale(p_.dir, 1.0 / n);
}

FixtureOutput VirtualFixture::apply(const Vec3& p, const Vec3& v_cmd) const
{
  FixtureOutput out;
  const Vec3 rel = sub(p, p_.origin);
  out.axial = dot(rel, p_.dir);
  const Vec3 on_axis = add(p_.origin, scale(p_.dir, out.axial));  // closest point on the axis
  const Vec3 radial_vec = sub(on_axis, p);                         // points back to the axis
  out.radial = norm(radial_vec);

  out.engaged = out.axial > -p_.capture_behind && out.radial < p_.capture_radius;
  if (!out.engaged)
  {
    out.v = v_cmd;
    return out;
  }

  double v_ax = dot(v_cmd, p_.dir);
  const Vec3 v_perp = sub(v_cmd, scale(p_.dir, v_ax));

  // Do not let the operator push forward while misaligned near or inside the port.
  if (v_ax > 0.0 && out.radial > p_.entry_tolerance && out.axial > -0.03)
  {
    v_ax = 0.0;
    out.forward_blocked = true;
  }

  out.v = add(add(scale(p_.dir, v_ax), scale(v_perp, p_.k_perp)), scale(radial_vec, p_.k_pull));
  return out;
}
}  // namespace rh_cell_core
