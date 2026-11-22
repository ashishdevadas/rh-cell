// Unit tests for rh_cell_core. No Gazebo, no ROS. They FAIL until TODO 1 and TODO 2 are done.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "rh_cell_core/admittance_1d.hpp"
#include "rh_cell_core/cartesian_admittance.hpp"
#include "rh_cell_core/peg_contact_2d.hpp"
#include "rh_cell_core/force_guard.hpp"
#include "rh_cell_core/insertion_monitor.hpp"
#include "rh_cell_core/virtual_fixture.hpp"
#include "rh_cell_core/signal.hpp"

using rh_cell_core::Admittance1D;
using rh_cell_core::ForceGuard;

TEST(Admittance, FollowsTargetInFreeSpace)
{
  Admittance1D ctrl({2.0, 60.0, 500.0}, 0.0);
  for (int i = 0; i < 1500; ++i) ctrl.step(0.05, 0.0, 0.002);
  EXPECT_NEAR(ctrl.position(), 0.05, 1e-3) << "Without contact the tool must reach x_des.";
}

TEST(Admittance, YieldsUnderConstantPush)
{
  Admittance1D ctrl({2.0, 60.0, 500.0}, 0.0);
  for (int i = 0; i < 2500; ++i) ctrl.step(0.0, -5.0, 0.002);
  EXPECT_NEAR(ctrl.position(), -0.01, 5e-4) << "-5 N on a 500 N/m spring = 10 mm back.";
}

TEST(Admittance, ContactForceStaysLowAgainstStiffWall)
{
  const double dt = 0.002, k_env = 50000.0, start = -0.05, target = 0.01;
  Admittance1D ctrl({2.0, 60.0, 500.0}, start);
  double f = 0.0, peak = 0.0;
  for (int i = 0; i <= 4000; ++i)
  {
    const double x_des = std::min(target, start + 0.01 * i * dt);
    const double x = ctrl.step(x_des, f, dt);
    f = (x > 0.0) ? -k_env * x : 0.0;
    peak = std::max(peak, std::abs(f));
  }
  EXPECT_NEAR(std::abs(f), 5.0, 0.5) << "Steady contact force should be about K x 10 mm = 5 N.";
  EXPECT_LT(peak, 20.0) << "Peak contact force must stay under the 20 N guard limit.";
}

TEST(ForceGuard, SingleSpikeDoesNotTrip)
{
  ForceGuard g(20.0, 5);
  for (int i = 0; i < 4; ++i) g.update(25.0);
  g.update(10.0);
  for (int i = 0; i < 4; ++i) g.update(25.0);
  EXPECT_FALSE(g.tripped());
}

TEST(ForceGuard, SustainedOverloadTrips)
{
  ForceGuard g(20.0, 5);
  for (int i = 0; i < 5; ++i) g.update(-25.0);
  EXPECT_TRUE(g.tripped()) << "5 samples above the limit (either sign) must trip.";
}

TEST(ForceGuard, StaysTrippedUntilReset)
{
  ForceGuard g(20.0, 5);
  for (int i = 0; i < 5; ++i) g.update(25.0);
  g.update(0.0);
  EXPECT_TRUE(g.tripped()) << "An interlock must latch.";
  g.reset();
  EXPECT_FALSE(g.tripped());
}

// ---------------------------------------------------------------- signal helpers
TEST(Signal, LowPassReachesStep)
{
  rh_cell_core::LowPass lp(20.0);
  lp.update(0.0, 0.002);
  for (int i = 0; i < 500; ++i) lp.update(1.0, 0.002);
  EXPECT_NEAR(lp.value(), 1.0, 1e-3);
}

TEST(Signal, LowPassSmoothsSpike)
{
  rh_cell_core::LowPass lp(5.0);
  lp.update(0.0, 0.002);
  EXPECT_LT(lp.update(100.0, 0.002), 10.0) << "A one-sample spike must be attenuated.";
}

TEST(Signal, WatchdogTimesOut)
{
  rh_cell_core::Watchdog w(0.05);
  EXPECT_FALSE(w.alive(0.0)) << "Never fed = not alive.";
  w.feed(1.00);
  EXPECT_TRUE(w.alive(1.04));
  EXPECT_FALSE(w.alive(1.06));
}

// ---------------------------------------------------------------- insertion monitor
using rh_cell_core::InsertionMonitor;
using rh_cell_core::InsertionPhase;

// Most tests confirm contact on the first sample; the debounce has its own tests below.
rh_cell_core::InsertionMonitorParams noDebounce()
{
  rh_cell_core::InsertionMonitorParams p;
  p.contact_samples = 1;
  return p;
}

TEST(InsertionMonitor, FreeSpaceStaysFree)
{
  InsertionMonitor m({});
  for (double d = 0.0; d < 0.14; d += 0.001) m.update(d, 0.3);
  EXPECT_EQ(m.phase(), InsertionPhase::FREE);
}

TEST(InsertionMonitor, EarlyContactIsAFault)
{
  InsertionMonitor m(noDebounce());  // expected depth 0.15 m, tolerance 0.01 m
  m.update(0.03, 0.0);
  EXPECT_EQ(m.update(0.03, 4.0), InsertionPhase::EARLY_CONTACT) << "Touching at 30 mm means the port is not there.";
  EXPECT_EQ(m.update(0.15, 5.0), InsertionPhase::EARLY_CONTACT) << "A fault must latch.";
}

TEST(InsertionMonitor, SeatsAfterStableForce)
{
  InsertionMonitor m(noDebounce());
  for (int i = 0; i < 49; ++i) EXPECT_NE(m.update(0.152, 5.2), InsertionPhase::SEATED);
  EXPECT_EQ(m.update(0.152, 5.2), InsertionPhase::SEATED) << "50 samples inside 5 +/- 1 N = seated.";
}

TEST(InsertionMonitor, ForceOutsideBandDoesNotSeat)
{
  InsertionMonitor m({});
  for (int i = 0; i < 200; ++i) m.update(0.152, 9.0);
  EXPECT_EQ(m.phase(), InsertionPhase::CONTACT);
}

TEST(InsertionMonitor, ReboundAfterLegitContactIsNotEarly)
{
  InsertionMonitor m(noDebounce());
  m.update(0.150, 6.0);  // legitimate first touch at the seat
  EXPECT_EQ(m.update(0.130, 6.0), InsertionPhase::CONTACT) << "Bouncing back 20 mm after a real touch is not a misalignment.";
}

TEST(InsertionMonitor, JitterSpikesInFreeSpaceAreNotContact)
{
  InsertionMonitor m({});  // default: 10 samples to confirm
  // 2.1 N spikes for a few samples at 8 mm depth (seen in simulation from joint jitter)
  for (int k = 0; k < 20; ++k)
  {
    for (int i = 0; i < 4; ++i) m.update(0.008, 2.1);
    m.update(0.008, 0.3);
  }
  EXPECT_EQ(m.phase(), InsertionPhase::FREE);
}

TEST(InsertionMonitor, SustainedEarlyContactIsStillCaught)
{
  InsertionMonitor m({});
  InsertionPhase ph = InsertionPhase::FREE;
  for (int i = 0; i < 10; ++i) ph = m.update(0.03, 4.0);
  EXPECT_EQ(ph, InsertionPhase::EARLY_CONTACT) << "10 samples (40 ms) of real contact at 30 mm = fault.";
}

// ---------------------------------------------------------------- virtual fixture
using rh_cell_core::FixtureParams;
using rh_cell_core::VirtualFixture;

TEST(VirtualFixture, PassThroughOutsideZone)
{
  VirtualFixture vf(FixtureParams{});
  const auto out = vf.apply({-0.5, 0.3, 0.0}, {0.01, 0.02, 0.03});
  EXPECT_FALSE(out.engaged);
  EXPECT_DOUBLE_EQ(out.v[1], 0.02);
}

TEST(VirtualFixture, AttenuatesSidewaysAndPullsBack)
{
  FixtureParams p;  // axis = world x through the origin
  VirtualFixture vf(p);
  const auto out = vf.apply({-0.10, 0.002, 0.0}, {0.0, 0.05, 0.0});
  EXPECT_TRUE(out.engaged);
  // sideways: 0.1 * 0.05 = 0.005 m/s outwards, pull: 2.0 * 0.002 = 0.004 m/s back
  EXPECT_NEAR(out.v[1], 0.005 - 0.004, 1e-9);
}

TEST(VirtualFixture, BlocksForwardWhenMisalignedAtPort)
{
  VirtualFixture vf(FixtureParams{});
  const auto out = vf.apply({-0.01, 0.010, 0.0}, {0.02, 0.0, 0.0});
  EXPECT_TRUE(out.forward_blocked);
  EXPECT_LE(out.v[0], 0.0);
}

TEST(VirtualFixture, AllowsForwardWhenAligned)
{
  VirtualFixture vf(FixtureParams{});
  const auto out = vf.apply({-0.01, 0.001, 0.0}, {0.02, 0.0, 0.0});
  EXPECT_FALSE(out.forward_blocked);
  EXPECT_NEAR(out.v[0], 0.02, 1e-9);
}

TEST(InsertionMonitor, LeadInContactIsNotEarlyButLaterContactIs)
{
  auto p = noDebounce();
  p.lead_in_end = 0.055;  // chamfer contact expected up to 55 mm
  InsertionMonitor m(p);
  EXPECT_EQ(m.update(0.032, 5.0), rh_cell_core::InsertionPhase::FREE) << "sliding on the chamfer";
  EXPECT_EQ(m.update(0.080, 5.0), rh_cell_core::InsertionPhase::EARLY_CONTACT) << "blocked past the lead-in";
}

// ======================================================================= RH Cell 2.0, step 2
// Cartesian admittance + planar chamfer contact model (no Gazebo).
using rh_cell_core::CartesianAdmittance;
using rh_cell_core::CartesianAdmittanceParams;
using rh_cell_core::PegPortGeometry;

TEST(CartesianAdmittance, FollowsTargetInFreeSpace)
{
  CartesianAdmittance ctrl(CartesianAdmittanceParams{});
  const Eigen::Vector3d target(0.05, 0.004, -0.003);
  for (int i = 0; i < 2000; ++i) ctrl.step(target, Eigen::Vector3d::Zero(), 0.002);
  EXPECT_LT((ctrl.position() - target).norm(), 1e-3);
}

TEST(CartesianAdmittance, YieldsPerAxisWithItsOwnStiffness)
{
  CartesianAdmittance ctrl(CartesianAdmittanceParams{});  // K = 500 axial, 200 lateral
  for (int i = 0; i < 3000; ++i) ctrl.step(Eigen::Vector3d::Zero(), Eigen::Vector3d(-5.0, 1.0, 0.0), 0.002);
  EXPECT_NEAR(ctrl.position().x(), -0.010, 5e-4) << "-5 N on 500 N/m = 10 mm back";
  EXPECT_NEAR(ctrl.position().y(), 0.005, 5e-4) << "1 N on 200 N/m = 5 mm sideways";
  EXPECT_NEAR(ctrl.position().z(), 0.0, 1e-6);
}

TEST(CartesianAdmittance, DeflectionIsClamped)
{
  CartesianAdmittance ctrl(CartesianAdmittanceParams{});  // lateral limit 10 mm
  for (int i = 0; i < 3000; ++i) ctrl.step(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 100.0, 0.0), 0.002);
  EXPECT_NEAR(ctrl.position().y(), 0.010, 1e-9) << "100 N would ask for 0.5 m: must stop at the 10 mm limit";
}

namespace
{
struct ChamferRun
{
  bool inserted = false;
  double peak = 0.0;
  double max_lateral_move = 0.0;
};

// Believed port at y = offset (true port at y = 0). Reference: straight in at 5 mm/s along the
// believed axis. lateral_stiff = robot holds y exactly (B1/B2 behaviour).
ChamferRun simulateChamfer(double offset, double k_lat, double mu, bool lateral_stiff)
{
  PegPortGeometry g;
  g.mu = mu;
  CartesianAdmittanceParams p;
  p.stiffness = {500.0, k_lat, k_lat};
  CartesianAdmittance adm(p, Eigen::Vector3d(-0.005, offset, 0.0));
  rh_cell_core::PegPortContact contact(g);
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  ChamferRun r;
  const double dt = 0.002;
  for (int i = 0; i < 10000; ++i)
  {
    const double t = i * dt;
    const Eigen::Vector3d x_des(std::min(-0.005 + 0.005 * t, 0.035), offset, 0.0);
    Eigen::Vector3d x = adm.step(x_des, f, dt);
    if (lateral_stiff)
      x.y() = offset;
    const Eigen::Vector2d F = contact.force({x.x(), x.y()});
    f = Eigen::Vector3d(F.x(), F.y(), 0.0);
    r.peak = std::max(r.peak, F.norm());
    r.max_lateral_move = std::max(r.max_lateral_move, std::abs(x.y() - offset));
    if (x.x() > g.thickness)
    {
      r.inserted = true;
      break;
    }
  }
  return r;
}
}  // namespace

TEST(Chamfer, CentredCassetteGoesStraightIn)
{
  const auto r = simulateChamfer(0.0, 200.0, 0.3, true);
  EXPECT_TRUE(r.inserted);
  EXPECT_LT(r.peak, 0.1);
}

TEST(Chamfer, LaterallyStiffRobotJamsOnTheChamfer)
{
  const auto r = simulateChamfer(0.004, 200.0, 0.3, true);
  EXPECT_FALSE(r.inserted) << "B1/B2: cannot move sideways, so the chamfer cannot guide it";
  EXPECT_GT(r.peak, 10.0);
}

TEST(Chamfer, LaterallyCompliantRobotSlidesIn)
{
  const auto r = simulateChamfer(0.004, 200.0, 0.3, false);
  EXPECT_TRUE(r.inserted) << "4 mm offset, mu 0.3: the chamfer pushes the cassette into the port";
  EXPECT_GT(r.max_lateral_move, 0.0019) << "it must have moved sideways by at least offset - clearance";
  EXPECT_LT(r.peak, 10.0);
}

TEST(Chamfer, CompliantRobotStillWedgesWhenFrictionExceedsChamferSlope)
{
  const auto r = simulateChamfer(0.004, 200.0, 1.2, false);
  EXPECT_FALSE(r.inserted) << "mu 1.2 > tan(45 deg): friction locks the cassette on the chamfer";
}

TEST(Chamfer, OffsetBeyondTheChamferHitsTheWallFace)
{
  const auto r = simulateChamfer(0.009, 200.0, 0.3, false);
  EXPECT_FALSE(r.inserted) << "9 mm > clearance + chamfer: compliance alone cannot help, needs search";
}

// Step 2b: the Gazebo failure reproduced. Rigid contact (5e6 N/m) seen through the F/T low-pass
// filter with one sample of delay, 250 Hz. The filter lag destabilises the lateral admittance
// (it runs into its 10 mm limit and the guard trips); a faster filter plus more damping is stable.
namespace
{
struct LagRun
{
  bool inserted = false;
  bool tripped = false;  // force guard: 5 filtered samples > 20 N
  double max_lat = 0.0;
};

LagRun simulateLaggedContact(double offset, double damping_lat, double cutoff_hz)
{
  constexpr double kPi = 3.14159265358979323846;
  PegPortGeometry g;
  g.k_env = 5.0e6;
  g.k_t = 5.0e6;
  CartesianAdmittanceParams p;
  p.damping = {60.0, damping_lat, damping_lat};
  CartesianAdmittance adm(p, Eigen::Vector3d(-0.003, 0.0, 0.0));
  rh_cell_core::PegPortContact contact(g);
  const double dt = 0.004, a = 1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt);
  Eigen::Vector2d filt(0.0, 0.0), delayed(0.0, 0.0);
  int over = 0;
  LagRun r;
  for (int i = 0; i < 5000; ++i)
  {
    const Eigen::Vector3d x_des(std::min(-0.003 + 0.005 * i * dt, 0.035), 0.0, 0.0);
    const Eigen::Vector3d x = adm.step(x_des, Eigen::Vector3d(delayed.x(), delayed.y(), 0.0), dt);
    const Eigen::Vector2d F = contact.force({x.x(), x.y() + offset});  // true port at -offset
    delayed = filt;
    filt += a * (F - filt);
    over = (filt.norm() > 20.0) ? over + 1 : 0;
    if (over >= 5)
      r.tripped = true;
    r.max_lat = std::max(r.max_lat, std::abs(x.y()));
    if (x.x() > g.thickness)
    {
      r.inserted = true;
      break;
    }
  }
  return r;
}
}  // namespace

TEST(StiffContact, SlowForceFilterWithLowDampingGoesUnstable)
{
  const auto r = simulateLaggedContact(0.004, 60.0, 20.0);  // the first Gazebo attempt
  EXPECT_TRUE(r.tripped || r.max_lat > 0.0099) << "expected the 10 mm limit or a guard trip, as in Gazebo";
}

TEST(StiffContact, FasterFilterWithHigherDampingStaysStable)
{
  for (const double off : {0.003, 0.004, 0.005, 0.006})
  {
    const auto r = simulateLaggedContact(off, 500.0, 50.0);
    EXPECT_TRUE(r.inserted) << "offset " << off;
    EXPECT_FALSE(r.tripped) << "offset " << off;
    EXPECT_LT(r.max_lat, 0.008) << "offset " << off;
  }
}
