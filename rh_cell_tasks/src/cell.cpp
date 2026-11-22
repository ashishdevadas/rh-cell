#include "rh_cell_tasks/cell.hpp"

#include <chrono>
#include <thread>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace rh_cell
{
Eigen::Isometry3d toolPose(double x, double y, double z)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << x, y, z;
  T.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  return T;
}

namespace
{
moveit_msgs::msg::CollisionObject makeBox(const std::string& id, const std::string& frame, double sx, double sy,
                                          double sz, double cx, double cy, double cz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = frame;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};
  geometry_msgs::msg::Pose pose;
  pose.position.x = cx;
  pose.position.y = cy;
  pose.position.z = cz;
  pose.orientation.w = 1.0;
  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

std::string readLatchedString(const std::string& topic, std::chrono::seconds timeout)
{
  auto helper = std::make_shared<rclcpp::Node>("rh_topic_fetch");
  std::string value;
  auto sub = helper->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).transient_local().reliable(),
      [&value](const std_msgs::msg::String::SharedPtr msg) { value = msg->data; });
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (value.empty() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(helper);
    std::this_thread::sleep_for(100ms);
  }
  return value;
}

std::string readMoveGroupParam(const std::string& name)
{
  auto helper = std::make_shared<rclcpp::Node>("rh_param_fetch");
  rclcpp::SyncParametersClient client(helper, "move_group");
  if (!client.wait_for_service(2s))
    return "";
  const auto params = client.get_parameters({name});
  if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    return "";
  return params[0].as_string();
}
}  // namespace

std::vector<moveit_msgs::msg::CollisionObject> buildCell(const std::string& frame)
{
  const double half = kPortSize / 2.0;
  const double yL0 = kWallYMin, yL1 = kPortY - half;
  const double yR0 = kPortY + half, yR1 = kWallYMax;
  const double zB0 = kWallZMin, zB1 = kPortZ - half;
  const double zT0 = kPortZ + half, zT1 = kWallZMax;
  const double wallH = kWallZMax - kWallZMin;
  const double wallZc = (kWallZMax + kWallZMin) / 2.0;

  std::vector<moveit_msgs::msg::CollisionObject> objs;
  objs.push_back(makeBox("floor", frame, 2.0, 2.0, 0.02, 0.0, 0.0, -0.011));
  objs.push_back(makeBox("wall_left", frame, kWallThick, yL1 - yL0, wallH, kPortX, (yL0 + yL1) / 2.0, wallZc));
  objs.push_back(makeBox("wall_right", frame, kWallThick, yR1 - yR0, wallH, kPortX, (yR0 + yR1) / 2.0, wallZc));
  objs.push_back(makeBox("wall_bottom", frame, kWallThick, kPortSize, zB1 - zB0, kPortX, kPortY, (zB0 + zB1) / 2.0));
  objs.push_back(makeBox("wall_top", frame, kWallThick, kPortSize, zT1 - zT0, kPortX, kPortY, (zT0 + zT1) / 2.0));
  objs.push_back(makeBox("seat", frame, 0.02, 0.15, 0.15, kSeatX + 0.01, kPortY, kPortZ));
  objs.push_back(makeBox("pedestal", frame, 0.10, 0.10, 0.31, 0.45, 0.25, 0.155));
  return objs;
}

bool fetchRobotDescription(const rclcpp::Node::SharedPtr& node)
{
  for (const std::string name : {"robot_description", "robot_description_semantic"})
  {
    std::string value = readLatchedString("/" + name, 20s);
    if (value.empty())
      value = readMoveGroupParam(name);
    if (value.empty())
    {
      RCLCPP_ERROR(node->get_logger(), "Could not get %s. Is sim.launch.py fully started?", name.c_str());
      return false;
    }
    if (node->has_parameter(name))
      node->set_parameter(rclcpp::Parameter(name, value));
    else
      node->declare_parameter(name, value);
  }
  return true;
}
}  // namespace rh_cell
