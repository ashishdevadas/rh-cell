// Remote-handling cell — Phase 1.3
// Teleoperated insertion with a guidance virtual fixture and a force limit.
//
//   ros2 launch rh_cell_bringup sim.launch.py
//   ros2 run rh_cell_tasks teleop_fixture
//   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rh/teleop_cmd
//   ros2 run rh_cell_tasks teleop_demo.py          # or: the scripted "sloppy operator"
//   ros2 run rh_cell_tasks teleop_fixture --ros-args -p use_fixture:=false   # same, fixture off
//
// Keys (teleop_twist_keyboard, holonomic): i / , = in / out of the port (+x / -x)
//                                            J / L (Shift) = sideways (+y / -y), t / b = up / down
// The operator drives the tool in world x/y/z, orientation is locked. Inside the capture zone in
// front of the port the virtual fixture (rh_cell_core/virtual_fixture.hpp) keeps the cassette on
// the port axis and refuses forward motion while it is off-axis. Forward motion is also cut when
// the axial contact force exceeds `force_limit` (the operator cannot crush the seat).
// Dead-man: no command for `cmd_timeout` -> the tool stops. Ctrl-C hands control back to MoveIt.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>

#include "rh_cell_core/virtual_fixture.hpp"
#include "rh_cell_tasks/cartesian_streamer.hpp"
#include "rh_cell_tasks/cell.hpp"
#include "rh_cell_tasks/ft_sensor.hpp"

using namespace std::chrono_literals;

namespace
{
std::atomic<bool> g_stop{false};
void onSigint(int) { g_stop = true; }

rh_cell_core::Vec3 toVec3(const Eigen::Vector3d& v) { return {v.x(), v.y(), v.z()}; }
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSigint);
  auto node = std::make_shared<rclcpp::Node>(
      "rh_teleop_fixture", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides(
                               {rclcpp::Parameter("use_sim_time", true)}));
  auto log = node->get_logger();
  auto param = [&](const std::string& n, auto def) {
    using T = decltype(def);
    if (!node->has_parameter(n))
      node->declare_parameter<T>(n, def);
    return node->get_parameter(n).get_value<T>();
  };
  const double rate = param("rate", 100.0);
  const double max_speed = param("max_speed", 0.03);       // m/s at full stick
  const double force_limit = param("force_limit", 8.0);    // N, axial push limit
  const double cmd_timeout = param("cmd_timeout", 0.3);    // s, dead-man
  const bool use_fixture = param("use_fixture", true);
  std::string log_dir = param("log_dir", std::string(""));
  if (log_dir.empty())
  {
    const char* home = std::getenv("HOME");
    log_dir = std::string(home ? home : "/tmp") + "/.ros/rh_cell_logs";
  }

  if (!rh_cell::fetchRobotDescription(node))
    return 1;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });

  int rc = 0;
  {
    moveit::planning_interface::MoveGroupInterface mg(node, "ur_manipulator");
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObjects(rh_cell::buildCell(mg.getPlanningFrame()));
    {
      // MoveIt plans with the stock UR model: attach the cassette Gazebo has on the tool.
      moveit_msgs::msg::AttachedCollisionObject cas;
      cas.link_name = "tool0";
      cas.object.id = "cassette";
      cas.object.header.frame_id = "tool0";
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = {rh_cell::kCasW, rh_cell::kCasW, rh_cell::kCasLen};
      geometry_msgs::msg::Pose pose;
      pose.position.z = rh_cell::kToolToCasFront - rh_cell::kCasLen / 2.0;
      pose.orientation.w = 1.0;
      cas.object.primitives.push_back(box);
      cas.object.primitive_poses.push_back(pose);
      cas.object.operation = moveit_msgs::msg::CollisionObject::ADD;
      cas.touch_links = {"tool0", "wrist_3_link", "flange", "cassette"};
      psi.applyAttachedCollisionObject(cas);
    }
    mg.setMaxVelocityScalingFactor(0.3);
    mg.setMaxAccelerationScalingFactor(0.3);

    // Start 15 cm in front of the port, 2 cm off to the side, so the fixture has something to do.
    const Eigen::Isometry3d start = rh_cell::toolPose(rh_cell::kPreInsertX - 0.12, rh_cell::kPortY + 0.02, rh_cell::kPortZ);
    auto planAndExecute = [&mg]() {
      for (int attempt = 0; attempt < 5; ++attempt)
      {
        mg.setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (static_cast<bool>(mg.plan(plan)))
          return static_cast<bool>(mg.execute(plan));
        std::this_thread::sleep_for(300ms);
      }
      return false;
    };
    // Taught joints to the pre-insert pose; the last 12 cm back + 2 cm sideways are streamed
    // (a pose goal would let MoveIt pick a random IK solution, see insertion_sequence.cpp).
    mg.setJointValueTarget(rh_cell::kReadyJoints);
    const bool at_ready = planAndExecute();
    mg.setJointValueTarget(rh_cell::kPreInsertJoints);
    if (!(at_ready && planAndExecute()))
    {
      RCLCPP_ERROR(log, "Could not reach the start pose");
      g_stop = true;
      rc = 1;
    }

    rh_cell::CartesianStreamer streamer(node, mg.getRobotModel());
    rh_cell::FtSensor ft(node, "/rh/ft");

    std::mutex cmd_mtx;
    Eigen::Vector3d cmd = Eigen::Vector3d::Zero();
    rclcpp::Time last_cmd(0, 0, RCL_ROS_TIME);
    auto cmd_sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "/rh/teleop_cmd", 10, [&](const geometry_msgs::msg::Twist::SharedPtr m) {
          std::lock_guard<std::mutex> lock(cmd_mtx);
          cmd = Eigen::Vector3d(std::clamp(m->linear.x, -1.0, 1.0), std::clamp(m->linear.y, -1.0, 1.0),
                                std::clamp(m->linear.z, -1.0, 1.0)) * max_speed;
          last_cmd = node->now();
        });
    auto state_pub = node->create_publisher<std_msgs::msg::String>("/rh/state", rclcpp::QoS(1).transient_local());

    // Port axis: tool0 position when the cassette front face is at the wall front face.
    rh_cell_core::FixtureParams fp;
    fp.origin = {rh_cell::kPortX - rh_cell::kWallThick / 2.0 - rh_cell::kToolToCasFront, rh_cell::kPortY, rh_cell::kPortZ};
    fp.dir = {1.0, 0.0, 0.0};
    const rh_cell_core::VirtualFixture fixture(fp);

    if (!g_stop && streamer.waitForJointStates(5.0) && ft.tare(0.5) && streamer.activate())
    {
      const double period = 1.0 / rate;
      // Stream from the pre-insert pose to the start pose (12 cm back, 2 cm sideways) in 4 s.
      Eigen::Isometry3d target = streamer.measuredPose();
      target.linear() = start.linear();
      {
        const Eigen::Vector3d p0 = target.translation();
        const rclcpp::Time t0 = node->now();
        rclcpp::Time prev_m = t0;
        while (rclcpp::ok() && !g_stop)
        {
          const rclcpp::Time now = node->now();
          const double dt = (now - prev_m).seconds();
          if (dt < period * 0.999)
          {
            std::this_thread::sleep_for(500us);
            continue;
          }
          prev_m = now;
          const double a = std::min(1.0, (now - t0).seconds() / 4.0);
          const double s = a * a * (3.0 - 2.0 * a);  // smooth start and stop
          target.translation() = p0 + s * (start.translation() - p0);
          streamer.step(target, dt);
          if (a >= 1.0)
            break;
        }
      }
      rclcpp::Time prev = node->now();
      double last_print = 0.0;
      std_msgs::msg::String st;
      RCLCPP_INFO(log, "Teleop active (fixture %s). Drive with teleop_twist_keyboard (cmd_vel:=/rh/teleop_cmd).",
                  use_fixture ? "ON" : "OFF");

      std::filesystem::create_directories(log_dir);
      char stamp[32];
      const std::time_t wall = std::time(nullptr);
      std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&wall));
      const std::string log_path = log_dir + "/teleop_" + (use_fixture ? "fixture" : "free") + "_" + stamp + ".csv";
      std::ofstream csv(log_path);
      csv << "t,cmd_x,cmd_y,cmd_z,out_x,out_y,out_z,x,y,z,axial,radial,engaged,blocked,push\n";
      const rclcpp::Time t_start = node->now();

      while (rclcpp::ok() && !g_stop)
      {
        const rclcpp::Time now = node->now();
        const double dt = (now - prev).seconds();
        if (dt < period * 0.999)
        {
          std::this_thread::sleep_for(500us);
          continue;
        }
        prev = now;
        ft.update(dt);

        Eigen::Vector3d v;
        {
          std::lock_guard<std::mutex> lock(cmd_mtx);
          const bool fresh = last_cmd.nanoseconds() > 0 && (now - last_cmd).seconds() < cmd_timeout;
          v = fresh ? cmd : Eigen::Vector3d::Zero();
        }
        if (!ft.alive())
          v.setZero();  // sensor lost: no motion

        // Always evaluate the fixture (for the axial/radial log); apply its output only if enabled.
        rh_cell_core::FixtureOutput fo = fixture.apply(toVec3(target.translation()), toVec3(v));
        if (!use_fixture)
        {
          fo.v = toVec3(v);
          fo.engaged = fo.forward_blocked = false;
        }
        Eigen::Vector3d v_out(fo.v[0], fo.v[1], fo.v[2]);

        // Force limit: pushing harder than force_limit into the seat -> no more forward motion,
        // and back off slightly (0.2 mm/s per N above the limit).
        const double push = -ft.axial();
        if (push > force_limit)
          v_out.x() = std::min(v_out.x(), -0.0002 * (push - force_limit));

        target.translation() += v_out * dt;
        streamer.step(target, dt);

        {
          const Eigen::Vector3d p = streamer.measuredPose().translation();
          char row[256];
          std::snprintf(row, sizeof(row), "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%.2f\n",
                        (now - t_start).seconds(), v.x(), v.y(), v.z(), v_out.x(), v_out.y(), v_out.z(), p.x(), p.y(),
                        p.z(), fo.axial, fo.radial, fo.engaged ? 1 : 0, fo.forward_blocked ? 1 : 0, push);
          csv << row;
        }

        const double t = now.seconds();
        if (t - last_print > 0.5)
        {
          last_print = t;
          char buf[160];
          std::snprintf(buf, sizeof(buf), "TELEOP fixture=%s%s axial=%.0fmm radial=%.1fmm push=%.1fN",
                        fo.engaged ? "ON" : "off", fo.forward_blocked ? " BLOCKED" : "", fo.axial * 1000,
                        fo.radial * 1000, push);
          st.data = buf;
          state_pub->publish(st);
          RCLCPP_INFO(log, "%s", buf);
        }
      }
      streamer.deactivate();
      csv.close();
      RCLCPP_INFO(log, "Log: %s", log_path.c_str());
      st.data = "IDLE";
      state_pub->publish(st);
      RCLCPP_INFO(log, "Teleop stopped, control handed back to the trajectory controller.");
    }
    else if (!g_stop)
    {
      RCLCPP_ERROR(log, "Setup failed (joint states / F/T / controller switch)");
      rc = 1;
    }
  }
  rclcpp::shutdown();
  spinner.join();
  return rc;
}
