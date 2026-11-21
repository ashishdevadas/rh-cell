#pragma once
// Small signal-conditioning helpers for the F/T sensor. No ROS.

#include <cmath>

namespace rh_cell_core
{
// First-order low-pass filter:  y += alpha * (u - y),  alpha = dt / (tau + dt),  tau = 1 / (2*pi*fc)
class LowPass
{
public:
  explicit LowPass(double cutoff_hz) : cutoff_hz_(cutoff_hz) {}
  double update(double u, double dt)
  {
    if (!init_)
    {
      y_ = u;
      init_ = true;
      return y_;
    }
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz_);
    y_ += (dt / (tau + dt)) * (u - y_);
    return y_;
  }
  double value() const { return y_; }
  void reset() { init_ = false; }

private:
  double cutoff_hz_;
  double y_ = 0.0;
  bool init_ = false;
};

// Watchdog: the signal is "alive" if a sample arrived less than `timeout` seconds ago.
class Watchdog
{
public:
  explicit Watchdog(double timeout) : timeout_(timeout) {}
  void feed(double t)
  {
    last_ = t;
    fed_ = true;
  }
  bool alive(double now) const { return fed_ && (now - last_) <= timeout_; }
  double age(double now) const { return fed_ ? now - last_ : 1e9; }

private:
  double timeout_;
  double last_ = 0.0;
  bool fed_ = false;
};
}  // namespace rh_cell_core
