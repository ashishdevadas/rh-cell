#pragma once
// Shared cell geometry and ROS helpers for the Gazebo cell (Phase 1.2 onwards).
// Same numbers as rh_cell_description/worlds/rh_cell.sdf and the URDF. Robot base frame = world.

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rh_cell
{
// ---------------------------------------------------------------- geometry [m]
constexpr double kPortX = 0.60;         // wall centre plane (wall spans 0.59 .. 0.61)
constexpr double kWallThick = 0.02;
constexpr double kPortY = -0.25;        // port centre
constexpr double kPortZ = 0.35;
constexpr double kPortSize = 0.09;      // square opening
constexpr double kWallYMin = -0.55, kWallYMax = 0.05;
constexpr double kWallZMin = 0.10, kWallZMax = 0.60;
constexpr double kSeatX = 0.71;         // seat front face
constexpr double kCasLen = 0.12;
constexpr double kCasW = 0.07;
constexpr double kToolToCasFront = 0.005 + kCasLen;  // tool0 -> cassette front face along tool z

// tool0 x when the cassette front face is 30 mm before the wall, and when it touches the seat
constexpr double kPreInsertX = (kPortX - kWallThick / 2.0 - 0.03) - kToolToCasFront;  // 0.435
constexpr double kSeatedX = kSeatX - kToolToCasFront;                                 // 0.585
constexpr double kInsertDepth = kSeatedX - kPreInsertX;                               // 0.150

const std::vector<double> kReadyJoints{0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
// Taught joint configuration for tool0 at the pre-insert pose (same as kPreInsertQ in
// insertion_sequence.cpp): a pose goal would let MoveIt pick any IK solution, and some of them
// cannot make the straight insertion.
const std::vector<double> kPreInsertJoints{-0.9654, -1.1935, 2.1005, -4.0620, -0.6020, 1.5788};

// tool0 pose with its z-axis along world +x (insertion direction): +90 deg about y.
Eigen::Isometry3d toolPose(double x, double y, double z);

// Wall, seat, pedestal, floor as MoveIt collision objects. force_insertion attaches the cassette to tool0.
std::vector<moveit_msgs::msg::CollisionObject> buildCell(const std::string& frame);

// Copy robot_description and robot_description_semantic into `node` as parameters
// (from the latched topics, or from move_group). Needed before building a MoveGroupInterface.
bool fetchRobotDescription(const rclcpp::Node::SharedPtr& node);
}  // namespace rh_cell
