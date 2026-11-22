#pragma once
// Insertion monitor: decides, from depth and axial force, where the insertion stands.
//
//   FREE           no contact yet
//   CONTACT        touching the seat at the expected depth, force building up
//   SEATED         force has stayed inside the seating band long enough -> done
//   EARLY_CONTACT  contact well before the expected depth -> the port is not where we think
//                  (misalignment, obstacle). This is a FAULT: stop and back out.
//
// Depth = distance travelled along the insertion axis from the pre-insert pose [m].
// Force = magnitude of the axial contact force [N] (always >= 0 here).
// No ROS here on purpose (unit-testable).

namespace rh_cell_core
{
enum class InsertionPhase
{
  FREE,
  CONTACT,
  SEATED,
  EARLY_CONTACT
};

const char* toString(InsertionPhase p);

struct InsertionMonitorParams
{
  double expected_depth = 0.15;    // depth at which the cassette should touch the seat [m]
  double depth_tolerance = 0.01;   // contact closer than expected_depth - tolerance = early [m]
  double contact_threshold = 2.0;  // force that counts as contact [N]
  int contact_samples = 10;        // consecutive samples above threshold to confirm a touch
                                   // (40 ms at 250 Hz: rejects inertial jitter spikes in free space)
  double seat_force = 5.0;         // target seating force [N]
  double seat_band = 1.0;          // |force - seat_force| < band counts as seated [N]
  int seat_samples = 50;           // consecutive samples inside the band to declare SEATED
  double lead_in_end = 0.0;        // RH Cell 2.0: contact at depth < lead_in_end is EXPECTED (the
                                   // cassette sliding down the port chamfer), not early. 0 = off.
};

class InsertionMonitor
{
public:
  explicit InsertionMonitor(const InsertionMonitorParams& p);
  InsertionPhase update(double depth, double axial_force_abs);
  InsertionPhase phase() const { return phase_; }
  void reset();

private:
  InsertionMonitorParams p_;
  InsertionPhase phase_ = InsertionPhase::FREE;
  int in_band_ = 0;
  int touching_ = 0;
};
}  // namespace rh_cell_core
