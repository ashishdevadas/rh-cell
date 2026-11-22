#include "rh_cell_tasks/cartesian_streamer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

namespace rh_cell
{
CartesianStreamer::CartesianStreamer(const rclcpp::Node::SharedPtr& node,
                                     const moveit::core::RobotModelConstPtr& model, const std::string& group,
                                     const std::string& tip)
  : node_(node), model_(model), jmg_(model->getJointModelGroup(group)), tip_(tip), state_(model)
{
  joint_names_ = jmg_->getVariableNames();  // shoulder_pan, shoulder_lift, elbow, wrist_1..3
  q_meas_.assign(joint_names_.size(), 0.0);
  q_cmd_ = q_meas_;
  state_.setToDefaultValues();

  js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        // joint_states order is not the controller order (elbow comes first): map by name
        for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
        {
          const auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);
          if (it != joint_names_.end())
            q_meas_[static_cast<std::size_t>(it - joint_names_.begin())] = msg->position[i];
        }
        have_js_ = true;
      });
  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
  switch_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
}

bool CartesianStreamer::switchControllers(const std::vector<std::string>& on, const std::vector<std::string>& off)
{
  if (!switch_client_->wait_for_service(5s))
  {
    RCLCPP_ERROR(node_->get_logger(), "controller_manager/switch_controller not available");
    return false;
  }
  auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  req->activate_controllers = on;
  req->deactivate_controllers = off;
  req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  req->activate_asap = true;
  auto fut = switch_client_->async_send_request(req);
  if (fut.wait_for(10s) != std::future_status::ready || !fut.get()->ok)
  {
    RCLCPP_ERROR(node_->get_logger(), "Controller switch failed");
    return false;
  }
  return true;
}

bool CartesianStreamer::activate()
{
  syncToMeasured();
  publish();  // first command = where the robot already is, so nothing jumps
  if (!switchControllers({"forward_position_controller"}, {"scaled_joint_trajectory_controller"}))
    return false;
  for (int i = 0; i < 5; ++i)
  {
    publish();
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

bool CartesianStreamer::deactivate()
{
  hold();
  return switchControllers({"scaled_joint_trajectory_controller"}, {"forward_position_controller"});
}

bool CartesianStreamer::waitForJointStates(double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (have_js_)
        return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return false;
}

std::vector<double> CartesianStreamer::measured()
{
  std::lock_guard<std::mutex> lock(mtx_);
  return q_meas_;
}

void CartesianStreamer::syncToMeasured() { q_cmd_ = measured(); }

void CartesianStreamer::publish()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = q_cmd_;
  cmd_pub_->publish(msg);
}

void CartesianStreamer::hold() { publish(); }

bool CartesianStreamer::step(const Eigen::Isometry3d& target, double dt)
{
  state_.setJointGroupPositions(jmg_, q_cmd_);
  state_.updateLinkTransforms();
  const Eigen::Isometry3d T = state_.getGlobalLinkTransform(tip_);

  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = target.translation() - T.translation();
  const Eigen::AngleAxisd aa(target.linear() * T.linear().transpose());
  e.tail<3>() = aa.angle() * aa.axis();

  Eigen::MatrixXd J;
  state_.getJacobian(jmg_, state_.getLinkModel(tip_), Eigen::Vector3d::Zero(), J);
  const Eigen::MatrixXd JJt = J * J.transpose() + lambda_ * lambda_ * Eigen::MatrixXd::Identity(6, 6);
  const Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(e);

  const double max_step = max_joint_speed_ * dt;
  for (std::size_t i = 0; i < q_cmd_.size(); ++i)
    q_cmd_[i] += std::clamp(dq(static_cast<Eigen::Index>(i)), -max_step, max_step);
  publish();
  return true;
}

Eigen::Isometry3d CartesianStreamer::measuredPose()
{
  state_.setJointGroupPositions(jmg_, measured());
  state_.updateLinkTransforms();
  return state_.getGlobalLinkTransform(tip_);
}

Eigen::Isometry3d CartesianStreamer::commandedPose()
{
  state_.setJointGroupPositions(jmg_, q_cmd_);
  state_.updateLinkTransforms();
  return state_.getGlobalLinkTransform(tip_);
}

double CartesianStreamer::maxTrackingError()
{
  const auto q = measured();
  double m = 0.0;
  for (std::size_t i = 0; i < q.size(); ++i)
    m = std::max(m, std::abs(q_cmd_[i] - q[i]));
  return m;
}
}  // namespace rh_cell
