// Remote-handling cell — Phase 1.2 + Phase 3: force-controlled insertion with fault injection
// The cassette (fixed to the tool in Gazebo) is pushed through the port until it seats on the
// spring-loaded backstop, held, and extracted.
//
//   ros2 launch rh_cell_bringup sim.launch.py      # terminal 1
//   ros2 run rh_cell_tasks force_insertion         # terminal 2 (nominal)
//   ros2 run rh_cell_tasks plot_run.py             # plot the latest log
//
// Fault injection (Phase 3): -p fault:=...
//   misaligned      the robot believes the port is 15 mm further along +y than it is (calibration
//                   error / arm deflection): the cassette hits the wall face instead of the opening
//   stiff           admittance switched off: pure position control into the seat
//   sensor_dropout  the F/T stream stops 4 s into the insertion
//
// Interlock (Phase 2): -p require_permit:=true  (start the sim with ic:=true)
//   The node waits for the PLC motion permit before any motion, publishes a 10 Hz heartbeat and
//   its force-guard / sensor status for the PLC, stops MoveIt trajectories when the permit drops,
//   and freezes the streamed insertion in place. After an e-stop, door or heartbeat trip nothing
//   moves until the operator resets; after a fault the controller detected itself (force, sensor)
//   it may back the cassette out on its own.
//
// Sequence
//   1. MoveIt: ready -> pre-insert (taught joint configuration, like Phase 1.1)
//   2. Tare the F/T sensor at rest (removes the cassette weight in this orientation)
//   3. Switch scaled_joint_trajectory_controller -> forward_position_controller
//   4. 250 Hz loop (sim time):
//        s_des : reference depth along the port axis (ramp: fast in free space, slow through the
//                port mouth and near the seat, ends `seat_overshoot` BEHIND the seat face on purpose)
//        s_cmd : admittance output   M*s'' + D*s' + K*(s - s_des) = f_axial
//        pose  : pre-insert pose shifted by s_cmd along +x -> CartesianStreamer -> joint positions
//   5. Seated (force steady in the seating band) -> hold -> extract -> switch back -> ready
// Safety layer (any of these stops the insertion and backs the cassette out):
//   ForceGuard        |F| > force_limit for 5 samples             FAULT_FORCE_LIMIT
//   InsertionMonitor  contact before the expected depth           FAULT_EARLY_CONTACT
//   Watchdog          no F/T sample for 50 ms                     FAULT_SENSOR
//   Tracking check    |q_cmd - q_meas| > 0.15 rad (robot blocked)  FAULT_TRACKING (stop, no retract)

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "rh_cell_core/admittance_1d.hpp"
#include "rh_cell_core/force_guard.hpp"
#include "rh_cell_core/insertion_monitor.hpp"
#include "rh_cell_tasks/cartesian_streamer.hpp"
#include "rh_cell_tasks/cell.hpp"
#include "rh_cell_tasks/ft_sensor.hpp"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

namespace
{
struct Params
{
  std::string fault = "none";
  double misalign_y = 0.015;    // [m] belief error for fault:=misaligned
  double dropout_after = 4.0;   // [s] for fault:=sensor_dropout
  double mass = 2.0, damping = 60.0, stiffness = 500.0;  // admittance (D = 60: see docs/contact_force_D200.png)
  double seat_overshoot = 0.01;  // reference ends 10 mm behind the seat face -> bounded push
  double force_limit = 20.0;
  double contact_threshold = 2.0;
  double v_fast = 0.02, v_slow = 0.005, slow_zone = 0.02;  // m/s, m/s, m before the seat
  // Port mouth: the cassette front face reaches the wall at s = 30 mm. Pass it slowly.
  double mouth_start = 0.015, mouth_end = 0.05;
  double v_retract = 0.02;
  double rate = 250.0;
  double hold_time = 2.0;
  double seat_timeout = 15.0;
  bool require_permit = false;  // Phase 2: obey the PLC motion permit
  double permit_timeout = 120.0;
  std::string log_dir;
};

class ForceInsertion
{
public:
  explicit ForceInsertion(const rclcpp::Node::SharedPtr& node)
    : node_(node), log_(node->get_logger()), mg_(node, "ur_manipulator")
  {
    readParams();
    mg_.setEndEffectorLink("tool0");
    mg_.setPlanningTime(5.0);
    mg_.setNumPlanningAttempts(5);
    mg_.setMaxVelocityScalingFactor(0.3);
    mg_.setMaxAccelerationScalingFactor(0.3);
    streamer_ = std::make_unique<rh_cell::CartesianStreamer>(node_, mg_.getRobotModel());
    ft_ = std::make_unique<rh_cell::FtSensor>(node_, "/rh/ft");
    state_pub_ = node_->create_publisher<std_msgs::msg::String>("/rh/state", rclcpp::QoS(1).transient_local());

    // ---- Phase 2: status for the PLC bridge / EPICS IOC, permit from the PLC
    guard_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/rh/guard_tripped", 10);
    alive_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/rh/ft_alive", 10);
    depth_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/rh/depth", 10);
    const auto latched = rclcpp::QoS(1).transient_local();
    permit_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/rh/permit", latched, [this](const std_msgs::msg::Bool::SharedPtr m) {
          const bool was = permit_.exchange(m->data);
          if (p_.require_permit && was && !m->data && moveit_busy_)
          {
            RCLCPP_WARN(log_, "Permit withdrawn during a MoveIt trajectory: stopping");
            mg_.stop();
          }
        });
    first_out_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "/rh/first_out", latched, [this](const std_msgs::msg::Int32::SharedPtr m) { first_out_ = m->data; });
    heartbeat_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/rh/heartbeat", 10);
    heartbeat_timer_ = node_->create_wall_timer(100ms, [this] {
      std_msgs::msg::Bool b;
      b.data = true;
      heartbeat_pub_->publish(b);
    });
  }

  int run()
  {
    setState("SETUP");
    if (!setupScene())
      return fail("SETUP_FAILED");
    if (p_.require_permit && !waitForPermit())
      return fail("NO_PERMIT");
    if (!moveJoints(rh_cell::kReadyJoints))
      return fail("PLAN_FAILED");
    setState("MOVE_TO_PORT");
    if (!moveJoints(rh_cell::kPreInsertJoints))
      return fail("PLAN_FAILED");
    start_ = rh_cell::toolPose(rh_cell::kPreInsertX, rh_cell::kPortY, rh_cell::kPortZ);

    if (!streamer_->waitForJointStates(5.0) || ft_->samples() == 0)
      return fail("NO_SENSOR_DATA");
    std::this_thread::sleep_for(500ms);  // let the arm settle before zeroing the sensor
    if (!ft_->tare(0.5))
      return fail("TARE_FAILED");
    if (!streamer_->activate())
      return fail("CONTROLLER_SWITCH_FAILED");

    openLog();
    const std::string result = insertLoop();
    const bool stopped = (result == "FAULT_TRACKING");
    if (!stopped)
      retract(result == "SEATED" ? p_.v_retract : 0.01);
    streamer_->deactivate();
    log_file_.close();
    if (!stopped)
    {
      if (p_.require_permit && !permit_)
        waitForPermit();  // after a trip the operator must reset before the arm moves again
      moveJoints(rh_cell::kReadyJoints);
    }

    setState(result == "SEATED" ? "DONE" : result);
    RCLCPP_INFO(log_, "RESULT: %s | peak |F| %.1f N | final axial %.1f N | max depth %.1f mm | %.1f s | log %s",
                result.c_str(), peak_, final_axial_, max_depth_ * 1000.0, cycle_time_, log_path_.c_str());
    return result == "SEATED" ? 0 : 2;
  }

private:
  void readParams()
  {
    auto get = [this](const std::string& n, auto def) {
      using T = decltype(def);
      if (!node_->has_parameter(n))
        node_->declare_parameter<T>(n, def);
      return node_->get_parameter(n).get_value<T>();
    };
    p_.fault = get("fault", p_.fault);
    p_.misalign_y = get("misalign_y", p_.misalign_y);
    p_.dropout_after = get("dropout_after", p_.dropout_after);
    if (p_.fault != "none" && p_.fault != "misaligned" && p_.fault != "stiff" && p_.fault != "sensor_dropout")
    {
      RCLCPP_WARN(log_, "Unknown fault '%s' (use misaligned, stiff, sensor_dropout): running nominal.",
                  p_.fault.c_str());
      p_.fault = "none";
    }
    p_.require_permit = get("require_permit", p_.require_permit);
    p_.permit_timeout = get("permit_timeout", p_.permit_timeout);
    p_.mass = get("mass", p_.mass);
    p_.damping = get("damping", p_.damping);
    p_.stiffness = get("stiffness", p_.stiffness);
    p_.seat_overshoot = get("seat_overshoot", p_.seat_overshoot);
    p_.force_limit = get("force_limit", p_.force_limit);
    p_.contact_threshold = get("contact_threshold", p_.contact_threshold);
    p_.v_fast = get("v_fast", p_.v_fast);
    p_.v_slow = get("v_slow", p_.v_slow);
    p_.rate = get("rate", p_.rate);
    p_.hold_time = get("hold_time", p_.hold_time);
    p_.log_dir = get("log_dir", std::string(""));
    if (p_.log_dir.empty())
    {
      const char* home = std::getenv("HOME");
      p_.log_dir = std::string(home ? home : "/tmp") + "/.ros/rh_cell_logs";
    }
    RCLCPP_INFO(log_, "fault=%s | Admittance M=%.1f D=%.0f K=%.0f | force limit %.0f N | seat overshoot %.0f mm",
                p_.fault.c_str(), p_.mass, p_.damping, p_.stiffness, p_.force_limit, p_.seat_overshoot * 1000.0);
  }

  // ------------------------------------------------------------------ main control loop
  std::string insertLoop()
  {
    rh_cell_core::Admittance1D adm({p_.mass, p_.damping, p_.stiffness}, 0.0);
    rh_cell_core::ForceGuard guard(p_.force_limit, 5);
    rh_cell_core::InsertionMonitorParams mp;
    mp.expected_depth = rh_cell::kInsertDepth;
    mp.contact_threshold = p_.contact_threshold;
    mp.seat_force = p_.stiffness * p_.seat_overshoot;
    rh_cell_core::InsertionMonitor monitor(mp);

    const Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    const double s_end = rh_cell::kInsertDepth + p_.seat_overshoot;
    const double period = 1.0 / p_.rate;
    double s_des = 0.0;
    s_cmd_ = 0.0;
    bool seated = false, holding = false;
    double t_seated = 0.0, t_end_reached = -1.0;

    const rclcpp::Time t0 = node_->now();
    rclcpp::Time prev = t0;
    setState("INSERTING");

    while (rclcpp::ok())
    {
      const rclcpp::Time now = node_->now();
      const double dt = (now - prev).seconds();
      if (dt < period * 0.999)
      {
        std::this_thread::sleep_for(500us);
        continue;
      }
      prev = now;
      const double t = (now - t0).seconds();

      if (p_.fault == "sensor_dropout" && t > p_.dropout_after)
        ft_->setDropped(true);  // pretend the cable is cut
      if (p_.fault == "misaligned")
        offset_.y() = p_.misalign_y * std::min(1.0, t / 1.0);  // move to where the robot BELIEVES the port is
      ft_->update(dt);
      const bool alive = ft_->alive();
      const double f_ax = ft_->axial();  // force the environment applies along +insertion (contact < 0)

      // ---- interlock (Phase 2): freeze in place while the PLC withdraws the motion permit
      if (p_.require_permit && !permit_)
      {
        if (!holding)
        {
          RCLCPP_WARN(log_, "Motion permit withdrawn by the interlock: HOLD");
          setState("HOLD_INTERLOCK");
          holding = true;
        }
        streamer_->hold();
        publishStatus(guard_latched_, alive, depthMeasured(axis));
        continue;
      }
      if (holding)
        return "FAULT_INTERLOCK";  // permit is back: back out, the operator restarts the cycle

      if (!alive)
        return logFault("FAULT_SENSOR", "No F/T data for 50 ms");

      // ---- reference: fast in free space, slow through the port mouth and near the seat
      if (!seated)
      {
        const bool in_mouth = s_des > p_.mouth_start && s_des < p_.mouth_end;
        const bool near_seat = s_des > rh_cell::kInsertDepth - p_.slow_zone;
        const double v = (in_mouth || near_seat) ? p_.v_slow : p_.v_fast;
        s_des = std::min(s_des + v * dt, s_end);
        if (s_des >= s_end && t_end_reached < 0.0)
          t_end_reached = t;
      }

      // ---- admittance: the commanded depth gives way when the seat pushes back
      //      (fault:=stiff switches it off: pure position control)
      s_cmd_ = (p_.fault == "stiff") ? s_des : adm.step(s_des, f_ax, dt);
      Eigen::Isometry3d target = start_;
      target.translation() += s_cmd_ * axis + offset_;
      streamer_->step(target, dt);

      const double depth = depthMeasured(axis);
      const bool tripped = guard.update(ft_->magnitude());
      const auto phase = monitor.update(depth, std::max(0.0, -f_ax));
      peak_ = std::max(peak_, ft_->magnitude());
      max_depth_ = std::max(max_depth_, depth);
      final_axial_ = f_ax;
      t_last_ = t;
      logRow(t, s_des, s_cmd_, depth, alive, tripped, rh_cell_core::toString(phase));
      if (tripped)
        guard_latched_ = true;  // the guard latches; keep reporting it so the PLC cannot miss it
      publishStatus(guard_latched_, alive, depth);

      if (tripped)
        return logFault("FAULT_FORCE_LIMIT", "Force above limit for 5 samples");
      if (phase == rh_cell_core::InsertionPhase::EARLY_CONTACT)
        return logFault("FAULT_EARLY_CONTACT", "Contact before the expected depth: port misaligned?");
      if (streamer_->maxTrackingError() > 0.15)
        return logFault("FAULT_TRACKING", "Robot does not follow the command (blocked?)");

      if (!seated && phase == rh_cell_core::InsertionPhase::SEATED)
      {
        seated = true;
        t_seated = t;
        setState("SEATED");
        RCLCPP_INFO(log_, "Seated at depth %.1f mm with %.1f N", depth * 1000.0, -f_ax);
      }
      if (seated && t - t_seated > p_.hold_time)
      {
        cycle_time_ = t;
        return "SEATED";
      }
      if (!seated && t_end_reached > 0.0 && t - t_end_reached > p_.seat_timeout)
        return logFault("FAULT_NOT_SEATED", "Reference reached the end but the cassette never seated");
    }
    return "ABORTED";
  }

  // Back out along the axis to the pre-insert pose (position control, logged).
  void retract(double speed)
  {
    setState(state_ == "SEATED" || state_ == "INSERTING" ? "EXTRACTING" : "RETRACTING_" + state_);
    const Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    const double period = 1.0 / p_.rate;
    rclcpp::Time prev = node_->now();
    const rclcpp::Time t0 = prev;
    while (rclcpp::ok() && s_cmd_ > 0.0)
    {
      const rclcpp::Time now = node_->now();
      const double dt = (now - prev).seconds();
      if (dt < period * 0.999)
      {
        std::this_thread::sleep_for(500us);
        continue;
      }
      prev = now;
      ft_->update(dt);
      if (mustFreeze())
      {
        streamer_->hold();  // e-stop / door / heartbeat: no motion at all, not even retreat
        continue;
      }
      s_cmd_ = std::max(0.0, s_cmd_ - speed * dt);
      Eigen::Isometry3d target = start_;
      target.translation() += s_cmd_ * axis + offset_;
      streamer_->step(target, dt);
      const double depth = depthMeasured(axis);
      logRow(t_last_ + (now - t0).seconds(), s_cmd_, s_cmd_, depth, ft_->alive(), false, "RETRACT");
      publishStatus(guard_latched_, ft_->alive(), depth);
    }
    const rclcpp::Time t_end = node_->now();  // settle at the pre-insert pose
    Eigen::Isometry3d hold = start_;
    hold.translation() += offset_;
    while (rclcpp::ok() && (node_->now() - t_end).seconds() < 0.5)
    {
      streamer_->step(hold, period);
      std::this_thread::sleep_for(10ms);
    }
  }

  // ------------------------------------------------------------------ helpers
  // Interlock causes that forbid ANY motion (first-out codes from rh_cell_ic/interlock.py):
  // 1 e-stop, 2 door, 5 controller heartbeat, 6 PLC comms.
  bool mustFreeze() const
  {
    const int fo = first_out_;
    return p_.require_permit && !permit_ && (fo == 1 || fo == 2 || fo == 5 || fo == 6);
  }

  bool waitForPermit()
  {
    setState("WAIT_PERMIT");
    RCLCPP_INFO(log_, "Waiting for the interlock permit (press RESET on the operator screen) ...");
    const auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !permit_)
    {
      if (std::chrono::steady_clock::now() - t0 > std::chrono::duration<double>(p_.permit_timeout))
        return false;
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(log_, "Permit received");
    return permit_;
  }

  void publishStatus(bool tripped, bool alive, double depth)
  {
    std_msgs::msg::Bool b;
    b.data = tripped;
    guard_pub_->publish(b);
    b.data = alive;
    alive_pub_->publish(b);
    std_msgs::msg::Float64 d;
    d.data = depth;
    depth_pub_->publish(d);
  }

  double depthMeasured(const Eigen::Vector3d& axis)
  {
    return (streamer_->measuredPose().translation() - start_.translation()).dot(axis);
  }

  std::string logFault(const std::string& code, const std::string& why)
  {
    RCLCPP_ERROR(log_, "%s: %s (depth %.1f mm, |F| %.1f N)", code.c_str(), why.c_str(), max_depth_ * 1000.0,
                 ft_->magnitude());
    setState(code);
    cycle_time_ = 0.0;
    return code;
  }

  int fail(const std::string& code)
  {
    setState(code);
    RCLCPP_ERROR(log_, "%s", code.c_str());
    return 1;
  }

  void setState(const std::string& s)
  {
    state_ = s;
    std_msgs::msg::String m;
    m.data = s;
    state_pub_->publish(m);
  }

  void openLog()
  {
    std::filesystem::create_directories(p_.log_dir);
    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    log_path_ = p_.log_dir + "/run_" + p_.fault + "_" + stamp + ".csv";
    log_file_.open(log_path_);
    log_file_ << "t,s_des,s_cmd,depth,fx,fy,fz,f_axial,f_mag,ft_alive,guard,phase\n";
  }

  void logRow(double t, double s_des, double s_cmd, double depth, bool alive, bool guard, const char* phase)
  {
    if (!log_file_.is_open())
      return;
    const auto& w = ft_->wrench();
    char line[256];
    std::snprintf(line, sizeof(line), "%.4f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%s\n", t, s_des, s_cmd,
                  depth, w[0], w[1], w[2], ft_->axial(), ft_->magnitude(), alive ? 1 : 0, guard ? 1 : 0, phase);
    log_file_ << line;
  }

  // Cell as MoveIt collision objects, and the cassette attached to tool0: in this launch MoveIt
  // plans with the stock UR model, which does not know the cassette that Gazebo has on the tool.
  bool setupScene()
  {
    psi_.removeCollisionObjects({"floor", "wall_left", "wall_right", "wall_bottom", "wall_top", "seat", "pedestal"});
    std::this_thread::sleep_for(500ms);
    if (!psi_.applyCollisionObjects(rh_cell::buildCell(mg_.getPlanningFrame())))
      return false;

    moveit_msgs::msg::AttachedCollisionObject cas;
    cas.link_name = "tool0";
    cas.object.id = "cassette";
    cas.object.header.frame_id = "tool0";
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {rh_cell::kCasW, rh_cell::kCasW, rh_cell::kCasLen};
    geometry_msgs::msg::Pose pose;
    pose.position.z = rh_cell::kToolToCasFront - rh_cell::kCasLen / 2.0;  // 0.065: centre of the box
    pose.orientation.w = 1.0;
    cas.object.primitives.push_back(box);
    cas.object.primitive_poses.push_back(pose);
    cas.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    cas.touch_links = {"tool0", "wrist_3_link", "flange", "cassette"};
    return psi_.applyAttachedCollisionObject(cas);
  }

  bool planAndExecute()
  {
    for (int attempt = 1; attempt <= 5; ++attempt)
    {
      if (p_.require_permit && !permit_)
      {
        RCLCPP_ERROR(log_, "No interlock permit: trajectory refused");
        return false;
      }
      mg_.setStartStateToCurrentState();
      MoveGroupInterface::Plan plan;
      if (static_cast<bool>(mg_.plan(plan)))
      {
        moveit_busy_ = true;
        const bool ok = static_cast<bool>(mg_.execute(plan));
        moveit_busy_ = false;
        return ok;
      }
      RCLCPP_WARN(log_, "Planning attempt %d/5 failed", attempt);
      std::this_thread::sleep_for(300ms);
    }
    return false;
  }

  bool moveJoints(const std::vector<double>& q)
  {
    mg_.setJointValueTarget(q);
    return planAndExecute();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger log_;
  MoveGroupInterface mg_;
  moveit::planning_interface::PlanningSceneInterface psi_;
  std::unique_ptr<rh_cell::CartesianStreamer> streamer_;
  std::unique_ptr<rh_cell::FtSensor> ft_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr guard_pub_, alive_pub_, heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permit_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr first_out_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::atomic<bool> permit_{false};
  std::atomic<bool> moveit_busy_{false};
  std::atomic<int> first_out_{0};
  bool guard_latched_ = false;

  Params p_;
  Eigen::Isometry3d start_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d offset_ = Eigen::Vector3d::Zero();  // belief error (fault:=misaligned)
  std::string state_;
  double s_cmd_ = 0.0;
  double peak_ = 0.0, final_axial_ = 0.0, max_depth_ = 0.0, cycle_time_ = 0.0, t_last_ = 0.0;
  std::ofstream log_file_;
  std::string log_path_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "rh_force_insertion",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides(
          {rclcpp::Parameter("use_sim_time", true)}));
  if (!rh_cell::fetchRobotDescription(node))
  {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });

  int rc = 1;
  {
    ForceInsertion app(node);
    rc = app.run();
  }
  rclcpp::shutdown();
  spinner.join();
  return rc;
}
