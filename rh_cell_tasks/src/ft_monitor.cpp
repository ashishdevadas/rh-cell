// Live check of the F/T front-end (Step 3c-2a): tare, 20 Hz low-pass, 50 ms watchdog.
//
//   ros2 launch rh_cell_bringup sim.launch.py      # terminal 1
//   ros2 run rh_cell_tasks ft_monitor              # terminal 2
//
// 1. Averages the raw reading (should be the cassette weight, 4.9 N).
// 2. Tares for 1 s, then prints the tared, filtered force 5 times per second: ~0 N at rest.
// 3. Watchdog: stop the F/T bridge and the monitor reports the signal lost within 50 ms.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "rh_cell_tasks/ft_sensor.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "rh_ft_monitor", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  std::thread spinner([&ex] { ex.spin(); });
  auto log = node->get_logger();

  rh_cell::FtSensor ft(node, "/rh/ft");

  RCLCPP_INFO(log, "Waiting for /rh/ft ...");
  for (int i = 0; i < 200 && rclcpp::ok() && ft.samples() < 50; ++i) std::this_thread::sleep_for(50ms);
  if (ft.samples() < 50)
  {
    RCLCPP_ERROR(log, "No F/T data. Is the simulation running (ros2 launch rh_cell_bringup sim.launch.py)?");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  const auto r = ft.raw();
  RCLCPP_INFO(log, "Raw (before tare): F = [%.3f %.3f %.3f] N, |F| = %.3f N  (cassette weight 0.5 kg x 9.8 = 4.90 N)",
              r[0], r[1], r[2], std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]));

  RCLCPP_INFO(log, "Taring for 1 s (keep the robot still) ...");
  if (!ft.tare(1.0))
  {
    RCLCPP_ERROR(log, "Tare failed: not enough samples");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  const auto& b = ft.bias();
  RCLCPP_INFO(log, "Bias removed: F = [%.3f %.3f %.3f] N, T = [%.4f %.4f %.4f] N.m", b[0], b[1], b[2], b[3], b[4],
              b[5]);
  RCLCPP_INFO(log, "Tared + 20 Hz low-pass, printed every 0.2 s. Ctrl+C to stop.");

  rclcpp::Time prev = node->now();
  bool was_alive = true;
  int k = 0;
  while (rclcpp::ok())
  {
    std::this_thread::sleep_for(4ms);  // 250 Hz, like the force controller
    const rclcpp::Time now = node->now();
    const double dt = (now - prev).seconds();
    if (dt > 0.0)
    {
      ft.update(dt);
      prev = now;
    }
    const bool alive = ft.alive();
    if (was_alive && !alive)
      RCLCPP_ERROR(log, "WATCHDOG: F/T signal lost (no sample for > 50 ms) -> a controller must stop here");
    if (!was_alive && alive)
      RCLCPP_INFO(log, "F/T signal back");
    was_alive = alive;
    if (++k % 50 == 0 && alive)
    {
      const auto& w = ft.wrench();
      std::printf("t=%7.2f s  F = [%+7.3f %+7.3f %+7.3f] N  |F| = %6.3f N  axial = %+7.3f N\n", now.seconds(), w[0],
                  w[1], w[2], ft.magnitude(), ft.axial());
      std::fflush(stdout);
    }
  }
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
