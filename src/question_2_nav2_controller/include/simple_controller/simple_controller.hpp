#pragma once

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/*
 * Proportional Heading Controller Plugin for Nav2
 *
 * Plugin chain: Nav2 Controller Server --> SimpleController --> /cmd_vel
 *
 * How it works:
 *   - setPlan() receives the global path and resets current_waypoint_ to 0
 *   - computeVelocityCommands() runs at ~20 Hz and steers toward poses[current_waypoint_]
 *   - heading error  = atan2(dy, dx) - robot_yaw
 *   - linear.x       = linear_vel_ * cos(heading_err)
 *   - angular.z      = k_angular_  * heading_err
 *
 * Result: Robot steers through waypoints one by one, slowing forward speed
 *         when it needs to turn hard, stopping when the plan is exhausted
 */

namespace simple_controller
{

class SimpleController : public nav2_core::Controller
{
public:
  SimpleController() = default;
  ~SimpleController() override = default;

  // grab node handle, plugin name, and read parameters from server
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // fresh path from planner -- resets current_waypoint_ to 0
  void setPlan(const nav_msgs::msg::Path & path) override;

  // main control loop ~20 Hz -- returns linear.x and angular.z in base_link frame
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;


  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // atan2(2*(w*z + x*y),  1 - 2*(y² + z²))
  double getYaw(const geometry_msgs::msg::Quaternion & q);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  nav_msgs::msg::Path global_plan_;
  std::shared_ptr<tf2_ros::Buffer> tf_;    // needed to transform plan into odom frame

  double linear_vel_{0.3};     // forward speed (m/s)
  double k_angular_{1.5};      //
  size_t current_waypoint_{0};
};

}  // namespace simple_controller