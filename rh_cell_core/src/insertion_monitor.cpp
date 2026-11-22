#include "rh_cell_core/insertion_monitor.hpp"

#include <cmath>

namespace rh_cell_core
{
const char* toString(InsertionPhase p)
{
  switch (p)
  {
    case InsertionPhase::FREE: return "FREE";
    case InsertionPhase::CONTACT: return "CONTACT";
    case InsertionPhase::SEATED: return "SEATED";
    case InsertionPhase::EARLY_CONTACT: return "EARLY_CONTACT";
  }
  return "?";
}

InsertionMonitor::InsertionMonitor(const InsertionMonitorParams& p) : p_(p) {}

InsertionPhase InsertionMonitor::update(double depth, double f)
{
  // Terminal states latch until reset(), like the force guard.
  if (phase_ == InsertionPhase::SEATED || phase_ == InsertionPhase::EARLY_CONTACT)
    return phase_;

  // Lead-in zone (compliant controllers): touching the chamfer is part of the plan. The force guard
  // still bounds the force there; the monitor starts judging once the cassette is past it.
  if (depth < p_.lead_in_end)
  {
    in_band_ = 0;
    touching_ = 0;
    phase_ = InsertionPhase::FREE;
    return phase_;
  }

  const bool touching = f > p_.contact_threshold;
  if (!touching)
  {
    // No contact force: free space, or a brief dip while settling on the seat.
    in_band_ = 0;
    touching_ = 0;
    if (phase_ != InsertionPhase::CONTACT)
      phase_ = InsertionPhase::FREE;
    return phase_;
  }

  // Debounce the first touch: one noisy sample in free space is not a contact.
  if (phase_ == InsertionPhase::FREE && ++touching_ < p_.contact_samples)
    return phase_;

  // Judge "early" only at the FIRST touch. Once in legitimate contact, a small rebound that makes
  // the depth shallower is not a misalignment.
  if (phase_ == InsertionPhase::FREE && depth < p_.expected_depth - p_.depth_tolerance)
  {
    phase_ = InsertionPhase::EARLY_CONTACT;
    return phase_;
  }

  phase_ = InsertionPhase::CONTACT;
  in_band_ = (std::abs(f - p_.seat_force) < p_.seat_band) ? in_band_ + 1 : 0;
  if (in_band_ >= p_.seat_samples)
    phase_ = InsertionPhase::SEATED;
  return phase_;
}

void InsertionMonitor::reset()
{
  phase_ = InsertionPhase::FREE;
  in_band_ = 0;
  touching_ = 0;
}
}  // namespace rh_cell_core
