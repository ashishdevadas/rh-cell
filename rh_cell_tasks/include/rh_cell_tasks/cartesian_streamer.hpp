#pragma once
// Cartesian streaming: send a new tool0 pose every control period and turn it into joint
// positions for forward_position_controller.
//
// Each step does one damped-least-squares differential-IK update on the COMMANDED joints:
//
//     e  = [p_target - p(q_cmd) ; rotation error as angle*axis]      (6x1, world frame)
//     dq = J^T (J J^T + lambda^2 I)^-1 e                              (J = geometric Jacobian at q_cmd)
//     q_cmd += clamp(dq, max_joint_speed * dt)
//
// Same idea as q_dot = J^+ v from the MANIP labs, solved as a position update.
// MoveIt is used only for the robot model (FK and Jacobian), not for planning.

#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace rh_cell
{
class CartesianStreamer
{
public:
  CartesianStreamer(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& model,
                    const std::string& group = "ur_manipulator", const std::string& tip = "tool0");

  // Switch scaled_joint_trajectory_controller -> forward_position_controller and back.
  bool activate();
  bool deactivate();

  bool waitForJointStates(double timeout_s);
  // Start streaming from where the robot really is.
  void syncToMeasured();
  // One control period. Returns false if the step was rejected (e.g. tracking error too large).
  bool step(const Eigen::Isometry3d& target, double dt);
  // Re-send the current command (hold still).
  void hold();

  Eigen::Isometry3d measuredPose();
  Eigen::Isometry3d commandedPose();
  double maxTrackingError();  // max |q_cmd - q_meas| over joints [rad]

private:
  bool switchControllers(const std::vector<std::string>& on, const std::vector<std::string>& off);
  void publish();
  std::vector<double> measured();

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* jmg_;
  std::string tip_;
  moveit::core::RobotState state_;
  std::vector<std::string> joint_names_;
  std::vector<double> q_cmd_;

  std::mutex mtx_;
  std::vector<double> q_meas_;
  bool have_js_ = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;

  double lambda_ = 0.01;          // DLS damping
  double max_joint_speed_ = 1.0;  // rad/s
};
}  // namespace rh_cell
