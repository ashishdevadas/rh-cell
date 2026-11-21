#pragma once
// F/T sensor front-end: raw wrench from Gazebo -> tare (bias removal) -> low-pass -> watchdog.
//
// Frame: the sensor frame = tool0 orientation, z = insertion axis. The sensor reports the force
// the cassette applies on the robot. When the seat pushes the cassette back, the cassette pushes
// the robot back too, so fz < 0 on contact.
//
// Tare is only valid for the orientation it was taken in: gravity on the 0.5 kg cassette is
// re-distributed over x/y/z when the tool rotates. During the insertion the tool orientation is
// fixed, so tare once at the pre-insert pose.

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rh_cell_core/signal.hpp"

namespace rh_cell
{
class FtSensor
{
public:
  FtSensor(const rclcpp::Node::SharedPtr& node, const std::string& topic = "/rh/ft", double cutoff_hz = 20.0,
           double timeout_s = 0.05)
    : node_(node),
      lp_{rh_cell_core::LowPass(cutoff_hz), rh_cell_core::LowPass(cutoff_hz), rh_cell_core::LowPass(cutoff_hz),
          rh_cell_core::LowPass(cutoff_hz), rh_cell_core::LowPass(cutoff_hz), rh_cell_core::LowPass(cutoff_hz)},
      watchdog_(timeout_s)
  {
    sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
        topic, rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::WrenchStamped::SharedPtr m) {
          std::lock_guard<std::mutex> lock(mtx_);
          if (dropped_)
            return;  // fault injection: pretend the cable is cut
          raw_ = {m->wrench.force.x,  m->wrench.force.y,  m->wrench.force.z,
                  m->wrench.torque.x, m->wrench.torque.y, m->wrench.torque.z};
          last_rx_ = node_->now().seconds();
          ++count_;
        });
    pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(topic + "_filtered", 10);
  }

  // Average `seconds` of data as the zero point (removes the cassette weight and sensor bias).
  bool tare(double seconds)
  {
    const auto start = node_->now();
    std::array<double, 6> sum{};
    long n = 0;
    long last = samples();
    while (rclcpp::ok() && (node_->now() - start).seconds() < seconds)
    {
      const long c = samples();
      if (c != last)
      {
        const auto r = raw();
        for (int i = 0; i < 6; ++i) sum[i] += r[i];
        ++n;
        last = c;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    if (n < 5)
      return false;
    for (int i = 0; i < 6; ++i) bias_[i] = sum[i] / static_cast<double>(n);
    for (auto& f : lp_) f.reset();
    return true;
  }

  // Call once per control period. Updates the filtered, tared wrench and publishes it.
  void update(double dt)
  {
    const auto r = raw();
    for (int i = 0; i < 6; ++i) w_[i] = lp_[i].update(r[i] - bias_[i], dt);
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (last_rx_ > 0.0)
        watchdog_.feed(last_rx_);
    }
    geometry_msgs::msg::WrenchStamped m;
    m.header.stamp = node_->now();
    m.header.frame_id = "tool0";
    m.wrench.force.x = w_[0];
    m.wrench.force.y = w_[1];
    m.wrench.force.z = w_[2];
    m.wrench.torque.x = w_[3];
    m.wrench.torque.y = w_[4];
    m.wrench.torque.z = w_[5];
    pub_->publish(m);
  }

  double axial() const { return w_[2]; }  // along +z (insertion), contact < 0
  double lateral() const { return std::hypot(w_[0], w_[1]); }
  double magnitude() const { return std::sqrt(w_[0] * w_[0] + w_[1] * w_[1] + w_[2] * w_[2]); }
  const std::array<double, 6>& wrench() const { return w_; }
  const std::array<double, 6>& bias() const { return bias_; }
  bool alive() const { return watchdog_.alive(node_->now().seconds()); }

  void setDropped(bool d)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    dropped_ = d;
  }
  long samples()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return count_;
  }
  std::array<double, 6> raw()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return raw_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  std::mutex mtx_;
  std::array<double, 6> raw_{}, bias_{}, w_{};
  std::array<rh_cell_core::LowPass, 6> lp_;
  rh_cell_core::Watchdog watchdog_;
  double last_rx_ = 0.0;
  long count_ = 0;
  bool dropped_ = false;
};
}  // namespace rh_cell
