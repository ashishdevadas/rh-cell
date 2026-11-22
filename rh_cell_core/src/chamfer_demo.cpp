// RH Cell 2.0, step 2: why the robot needs lateral compliance, and when even that is not enough.
// Planar model (no Gazebo): a 70 mm cassette pushed at 5 mm/s into a 74 mm port with a 5 mm x 45 deg
// chamfer, starting from a lateral offset. Prints, for each offset, whether it got in and the peak
// contact force, for a laterally stiff robot (B1/B2) and for the Cartesian admittance controller
// with several friction coefficients and lateral stiffnesses.
//
//     ros2 run rh_cell_core chamfer_demo
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "rh_cell_core/cartesian_admittance.hpp"
#include "rh_cell_core/peg_contact_2d.hpp"

using rh_cell_core::CartesianAdmittance;
using rh_cell_core::CartesianAdmittanceParams;
using rh_cell_core::PegPortContact;
using rh_cell_core::PegPortGeometry;

namespace
{
bool run(double offset, double k_lat, double mu, bool lateral_stiff, double& peak)
{
  PegPortGeometry g;
  g.mu = mu;
  CartesianAdmittanceParams p;
  p.stiffness = {500.0, k_lat, k_lat};
  CartesianAdmittance adm(p, Eigen::Vector3d(-0.005, offset, 0.0));
  PegPortContact contact(g);
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  peak = 0.0;
  for (int i = 0; i < 10000; ++i)
  {
    const Eigen::Vector3d x_des(std::min(-0.005 + 0.005 * i * 0.002, 0.035), offset, 0.0);
    Eigen::Vector3d x = adm.step(x_des, f, 0.002);
    if (lateral_stiff)
      x.y() = offset;
    const Eigen::Vector2d F = contact.force({x.x(), x.y()});
    f = Eigen::Vector3d(F.x(), F.y(), 0.0);
    peak = std::max(peak, F.norm());
    if (x.x() > g.thickness)
      return true;
  }
  return false;
}

// Same task, but with what Gazebo adds: rigid contact (5e6 N/m), the F/T low-pass filter and one
// sample of delay at 250 Hz. Returns 'IN' / '--', 'T' if the force guard trips, max sideways move.
void lagged(double offset, double d_lat, double cutoff_hz, bool& in, bool& trip, double& max_lat)
{
  constexpr double kPi = 3.14159265358979323846;
  PegPortGeometry g;
  g.k_env = 5.0e6;
  g.k_t = 5.0e6;
  CartesianAdmittanceParams p;
  p.damping = {60.0, d_lat, d_lat};
  CartesianAdmittance adm(p, Eigen::Vector3d(-0.003, 0.0, 0.0));
  PegPortContact contact(g);
  const double dt = 0.004, a = 1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt);
  Eigen::Vector2d filt(0.0, 0.0), delayed(0.0, 0.0);
  int over = 0;
  in = trip = false;
  max_lat = 0.0;
  for (int i = 0; i < 5000; ++i)
  {
    const Eigen::Vector3d x_des(std::min(-0.003 + 0.005 * i * dt, 0.035), 0.0, 0.0);
    const Eigen::Vector3d x = adm.step(x_des, Eigen::Vector3d(delayed.x(), delayed.y(), 0.0), dt);
    const Eigen::Vector2d F = contact.force({x.x(), x.y() + offset});
    delayed = filt;
    filt += a * (F - filt);
    over = (filt.norm() > 20.0) ? over + 1 : 0;
    trip = trip || over >= 5;
    max_lat = std::max(max_lat, std::abs(x.y()));
    if (x.x() > g.thickness)
    {
      in = true;
      return;
    }
  }
}
}  // namespace

int main()
{
  struct Col
  {
    const char* name;
    double k_lat, mu;
    bool stiff;
  };
  const std::vector<Col> cols{{"stiff mu0.3", 200, 0.3, true},   {"K200 mu0.3", 200, 0.3, false},
                              {"K200 mu0.6", 200, 0.6, false},   {"K200 mu0.9", 200, 0.9, false},
                              {"K200 mu1.1", 200, 1.1, false},   {"K1000 mu0.3", 1000, 0.3, false}};
  std::printf("Cassette 70 mm, port 74 mm, chamfer 5 mm x 45 deg. Cell: IN/--- (peak contact force N)\n");
  std::printf("offset ");
  for (const auto& c : cols) std::printf(" %13s", c.name);
  std::printf("\n");
  for (const double mm : {0.0, 1.0, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 6.5, 7.0, 8.0})
  {
    std::printf("%4.1f mm", mm);
    for (const auto& c : cols)
    {
      double peak = 0.0;
      const bool in = run(mm * 1e-3, c.k_lat, c.mu, c.stiff, peak);
      std::printf("   %s (%5.1f)", in ? " IN" : "---", peak);
    }
    std::printf("\n");
  }
  std::printf("Stiff robot: only offsets inside the 2 mm clearance. Compliant: up to clearance + chamfer,\n"
              "unless mu > tan(45 deg) = 1 (wedging). Beyond the chamfer: the wall face - needs a search.\n");

  std::printf("\nWith rigid contact (5e6 N/m), F/T low-pass filter and 1 sample delay (250 Hz), mu 0.3, K 200 N/m.\n"
              "Cell: IN/-- , T = force guard trips, max sideways move [mm]\n");
  const double cfg[4][2] = {{60.0, 20.0}, {500.0, 20.0}, {60.0, 50.0}, {500.0, 50.0}};
  std::printf("offset    D60  20 Hz      D500 20 Hz      D60  50 Hz      D500 50 Hz\n");
  for (const double mm : {3.0, 4.0, 5.0, 6.0})
  {
    std::printf("%4.1f mm", mm);
    for (const auto& c : cfg)
    {
      bool in = false, trip = false;
      double lat = 0.0;
      lagged(mm * 1e-3, c[0], c[1], in, trip, lat);
      std::printf("   %s%s %5.1f mm", in ? "IN" : "--", trip ? "T" : " ", lat * 1e3);
    }
    std::printf("\n");
  }
  std::printf("Filter lag and low damping together destabilise the contact (10 mm limit or guard trip);\n"
              "only the 50 Hz filter with lateral damping 500 N*s/m slides in at every offset.\n");
  return 0;
}
