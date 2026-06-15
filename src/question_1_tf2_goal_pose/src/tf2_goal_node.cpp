#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

/*
 * TF2 Frame Management & 2D goal pose
 *
 * Frame chain: map --> odom --> base_link
 *
 * Question:
 *   - odom --> base_link is always identity (robot is fixed in odom frame)
 *   - When a goal is received, we shift map --> odom to that goal position
 *   - Since odom --> base_link is identity, base_link ends up at the goal in map frame
 *
 * Result: Robot visually jumps to wherever you click "2D Goal Pose" in RViz
 */

class TF2GoalNode : public rclcpp::Node
{
public:
  TF2GoalNode() : Node("tf2_goal_node")
  {
    // broadcaster to publish TF transforms
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // subscribe to the goal pose published by RViz "2D Goal Pose" button
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&TF2GoalNode::goalCallback, this, std::placeholders::_1));

    // map --> odom starts at identity (robot at origin)
    map_to_odom_.header.frame_id = "map";
    map_to_odom_.child_frame_id  = "odom";
    map_to_odom_.transform.translation.x = 0.0;
    map_to_odom_.transform.translation.y = 0.0;
    map_to_odom_.transform.translation.z = 0.0;
    map_to_odom_.transform.rotation.x    = 0.0;
    map_to_odom_.transform.rotation.y    = 0.0;
    map_to_odom_.transform.rotation.z    = 0.0;
    map_to_odom_.transform.rotation.w    = 1.0;

    // odom --> base_link is always identity and never changes
    odom_to_base_.header.frame_id = "odom";
    odom_to_base_.child_frame_id  = "base_link";
    odom_to_base_.transform.translation.x = 0.0;
    odom_to_base_.transform.translation.y = 0.0;
    odom_to_base_.transform.translation.z = 0.0;
    odom_to_base_.transform.rotation.x    = 0.0;
    odom_to_base_.transform.rotation.y    = 0.0;
    odom_to_base_.transform.rotation.z    = 0.0;
    odom_to_base_.transform.rotation.w    = 1.0;

    // broadcast both transforms at 10 Hz so RViz stays updated
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TF2GoalNode::broadcastTimer, this));

    RCLCPP_INFO(this->get_logger(), "Node started. Click '2D Goal Pose' in RViz to move the robot.");
  }

private:

  // publishes both transforms every 100ms with a fresh timestamp
  void broadcastTimer()
  {
    auto now = this->get_clock()->now();
    map_to_odom_.header.stamp  = now;
    odom_to_base_.header.stamp = now;

    tf_broadcaster_->sendTransform(map_to_odom_);
    tf_broadcaster_->sendTransform(odom_to_base_);
  }

  // triggered when user clicks "2D Goal Pose" in RViz
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Goal received -> x: %.2f, y: %.2f",
      msg->pose.position.x,
      msg->pose.position.y);

    // shift map --> odom to the goal position
    // because odom --> base_link is identity,
    // base_link will appear exactly at this goal in the map frame
    map_to_odom_.transform.translation.x = msg->pose.position.x;
    map_to_odom_.transform.translation.y = msg->pose.position.y;
    map_to_odom_.transform.translation.z = 0.0;

    // RViz sends a full quaternion for heading, use it directly
    map_to_odom_.transform.rotation = msg->pose.orientation;
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::TransformStamped map_to_odom_;
  geometry_msgs::msg::TransformStamped odom_to_base_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF2GoalNode>());
  rclcpp::shutdown();
  return 0;
}