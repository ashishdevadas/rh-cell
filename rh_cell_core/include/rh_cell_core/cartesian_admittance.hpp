#pragma once
// Cartesian (3-D translational) admittance controller, RH Cell 2.0 step 2.
//
//     M * a  +  D * v  +  K * (x - x_des)  =  f_ext        (per axis, diagonal gains)
//
// Same law as Admittance1D, on all three translations. Gains are given in the PORT frame
// (x = insertion axis, y / z = lateral), which in this cell is aligned with the world frame.
// f_ext = force the environment applies on the tool, in the same frame.
//
// Why lateral compliance matters: when the cassette lands on the lead-in chamfer, the chamfer
// pushes it sideways towards the port centre. A laterally stiff robot cannot follow that push,
// so the contact force climbs until something gives (jam, force limit). A laterally compliant
// robot slides down the chamfer into the port - the software version of a remote-centre-
// compliance (RCC) wrist. It only works if the chamfer is steeper than the friction cone:
// for a 45 deg chamfer the friction coefficient must be below tan(45 deg) = 1.
//
// Safety: the deflection |x - x_des| per axis is clamped to max_deflection, so a sensor fault
// or a large disturbance cannot drag the tool arbitrarily far from the reference.
// No ROS here (unit-testable); Eigen only.

#include <Eigen/Core>

namespace rh_cell_core
{
struct CartesianAdmittanceParams
{
  Eigen::Vector3d mass{2.0, 2.0, 2.0};               // M  [kg]       (x axial, y, z lateral)
  Eigen::Vector3d damping{60.0, 60.0, 60.0};         // D  [N*s/m]
  Eigen::Vector3d stiffness{500.0, 200.0, 200.0};    // K  [N/m]
  Eigen::Vector3d max_deflection{0.05, 0.01, 0.01};  // |x - x_des| limit per axis [m]
};

class CartesianAdmittance
{
public:
  explicit CartesianAdmittance(const CartesianAdmittanceParams& p,
                               const Eigen::Vector3d& x0 = Eigen::Vector3d::Zero());
  // One control period. Returns the commanded position.
  Eigen::Vector3d step(const Eigen::Vector3d& x_des, const Eigen::Vector3d& f_ext, double dt);
  void reset(const Eigen::Vector3d& x0);
  // Gains may change between steps (adaptive control, step 5).
  void setStiffness(const Eigen::Vector3d& k) { p_.stiffness = k; }
  void setDamping(const Eigen::Vector3d& d) { p_.damping = d; }
  const CartesianAdmittanceParams& params() const { return p_; }
  const Eigen::Vector3d& position() const { return x_; }
  const Eigen::Vector3d& velocity() const { return v_; }

private:
  CartesianAdmittanceParams p_;
  Eigen::Vector3d x_;
  Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
};
}  // namespace rh_cell_core
