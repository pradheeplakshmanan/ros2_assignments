#include "simple_controller/simple_controller.hpp"
#include <cmath>
#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"

/*
 * Proportional Heading Controller -- Implementation
 *
 * Control loop: global_plan_.poses[current_waypoint_] --> heading error --> cmd_vel
 *
 * How it works:
 *   - computeVelocityCommands() runs at ~20 Hz
 *   - each cycle we look at the active waypoint, compute dx/dy to it
 *   - if dist < 0.3m we advance current_waypoint_ and wait for next cycle
 *   - otherwise: desired_yaw = atan2(dy, dx)
 *                heading_err = shortest_angular_distance(robot_yaw, desired_yaw)
 *                linear.x    = linear_vel_ * cos(heading_err)   <-- slows when not facing target
 *                angular.z   = k_angular_  * heading_err        <-- P controller
 *
 * Result: Robot steers through waypoints one by one at roughly constant speed,
 *         naturally slowing forward motion when it needs to turn hard
 */

namespace simple_controller
{

void SimpleController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;    // ← save it

  auto node = node_.lock();
  RCLCPP_INFO(node->get_logger(),
    "SimpleController configured as '%s'", plugin_name_.c_str());

  // declare first, then get -- Nav2 pattern for plugin parameters
  node->declare_parameter(plugin_name_ + ".linear_velocity", 0.3);
  node->declare_parameter(plugin_name_ + ".k_angular", 1.5);

  node->get_parameter(plugin_name_ + ".linear_velocity", linear_vel_);
  node->get_parameter(plugin_name_ + ".k_angular", k_angular_);
}

void SimpleController::cleanup()
{
  auto node = node_.lock();
  RCLCPP_INFO(node->get_logger(), "SimpleController cleaning up");
}

void SimpleController::activate()
{
  auto node = node_.lock();
  RCLCPP_INFO(node->get_logger(), "SimpleController activated");
}

void SimpleController::deactivate()
{
  auto node = node_.lock();
  RCLCPP_INFO(node->get_logger(), "SimpleController deactivated");
}

// Nav2 calls this whenever the global planner produces a fresh path
void SimpleController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  current_waypoint_ = 0;  // reset progress
  auto node = node_.lock();

  RCLCPP_INFO(node->get_logger(),
    "New plan received with %zu waypoints", global_plan_.poses.size());

  RCLCPP_INFO(node->get_logger(),
    "First waypoint: (%.2f, %.2f)",
    path.poses.front().pose.position.x,
    path.poses.front().pose.position.y);

  RCLCPP_INFO(node->get_logger(),
    "Last waypoint: (%.2f, %.2f)",
    path.poses.back().pose.position.x,
    path.poses.back().pose.position.y);
}

// percentage=true  --> speed_limit is 0-100, scale current linear_vel_ by that %
// percentage=false --> speed_limit is absolute m/s, apply directly
void SimpleController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    linear_vel_ = linear_vel_ * speed_limit / 100.0;
  } else {
    linear_vel_ = speed_limit;
  }
}

geometry_msgs::msg::TwistStamped SimpleController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = pose.header.frame_id;
  cmd.header.stamp = pose.header.stamp;

  // no plan yet or walked every waypoint -- zero velocity stops the robot
  if (global_plan_.poses.empty() || current_waypoint_ >= global_plan_.poses.size()) {
    return cmd;
  }

  // vector from robot to active waypoint
  const auto & target_in_map = global_plan_.poses[current_waypoint_];

  // Transform the target waypoint from map frame into the robot's pose frame (odom)
  geometry_msgs::msg::PoseStamped target;
  try {
    tf_->transform(target_in_map, target, pose.header.frame_id,
                   tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_.lock()->get_logger(), "TF transform failed: %s", ex.what());
    return cmd;
  }

  double dx = target.pose.position.x - pose.pose.position.x;
  double dy = target.pose.position.y - pose.pose.position.y;
  double dist = std::hypot(dx, dy);

  // close enough -- next move toward the new waypoint
  if (dist < 0.3) {
    current_waypoint_++;   // ← advance through waypoints, not just stop
    if (current_waypoint_ >= global_plan_.poses.size()) {
      RCLCPP_INFO(node_.lock()->get_logger(), "Goal reached");
    }
    return cmd;
  }

  double robot_yaw   = getYaw(pose.pose.orientation);
  double desired_yaw = std::atan2(dy, dx);

  // shortest_angular_distance keeps the error in [-π, π]
  double heading_error = angles::shortest_angular_distance(robot_yaw, desired_yaw);

  RCLCPP_INFO(
    node_.lock()->get_logger(),
    "Robot=(%.2f, %.2f) Target=(%.2f, %.2f)",
    pose.pose.position.x,
    pose.pose.position.y,
    target.pose.position.x,
    target.pose.position.y);

  RCLCPP_INFO(
    node_.lock()->get_logger(),
    "RobotYaw=%.2f DesiredYaw=%.2f Error=%.2f",
    robot_yaw,
    desired_yaw,
    heading_error);

  RCLCPP_INFO(
    node_.lock()->get_logger(),
    "Pose frame=%s Plan frame=%s",
    pose.header.frame_id.c_str(),
    global_plan_.header.frame_id.c_str());

  RCLCPP_INFO(
  node_.lock()->get_logger(),
  "Current waypoint index: %zu / %zu",
  current_waypoint_,
  global_plan_.poses.size());

  if (std::fabs(heading_error) > 0.5) {
  cmd.twist.linear.x = 0.0;
  } else {
  cmd.twist.linear.x = linear_vel_;
  }


  cmd.twist.angular.z = k_angular_  * heading_error;

  return cmd;
}

// atan2(2*(w*z + x*y),  1 - 2*(y² + z²))
double SimpleController::getYaw(const geometry_msgs::msg::Quaternion & q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace simple_controller

// registers this class with pluginlib -- must match plugin.xml
PLUGINLIB_EXPORT_CLASS(simple_controller::SimpleController, nav2_core::Controller)