#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

/*
 * TF2 Frame Management & 2D goal pose
 *
 * Frame chain: map --> odom --> base_link
 *
 *   - odom --> base_link is not identity. I gave it a fixed value
 *     (0.7, 0.4) so it acts like the robot already has some odometry
 *     offset, like it would in a real robot.
 *   - When a new goal comes in, I calculate map --> odom so that
 *     when it's combined with odom --> base_link, base_link lands exactly
 *     on the goal.
 *
 *     map->odom = goal * (odom->base_link)^-1
 *
 *   - This way base_link goes to the clicked point, but odom stays a bit
 *     away from it (offset by 0.7, 0.4).
 */

class TF2GoalNode : public rclcpp::Node
{
public:
  TF2GoalNode() : Node("tf2_goal_node")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&TF2GoalNode::goalCallback, this, std::placeholders::_1));

    // map -> odom starts at zero, gets updated every time a goal is clicked
    map_to_odom_.header.frame_id = "map";
    map_to_odom_.child_frame_id  = "odom";
    map_to_odom_.transform.translation.x = 0.0;
    map_to_odom_.transform.translation.y = 0.0;
    map_to_odom_.transform.translation.z = 0.0;
    map_to_odom_.transform.rotation.x    = 0.0;
    map_to_odom_.transform.rotation.y    = 0.0;
    map_to_odom_.transform.rotation.z    = 0.0;
    map_to_odom_.transform.rotation.w    = 1.0;

    // odom -> base_link, fixed offset.
    odom_to_base_.header.frame_id = "odom";
    odom_to_base_.child_frame_id  = "base_link";
    odom_to_base_.transform.translation.x = 0.7;
    odom_to_base_.transform.translation.y = 0.4;
    odom_to_base_.transform.translation.z = 0.0;
    odom_to_base_.transform.rotation.x    = 0.0;
    odom_to_base_.transform.rotation.y    = 0.0;
    odom_to_base_.transform.rotation.z    = 0.0;
    odom_to_base_.transform.rotation.w    = 1.0;

    // keep sending both transforms every 100ms so RViz keeps showing them
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TF2GoalNode::broadcastTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "Node started. Click '2D Goal Pose' in RViz to move base_link there.");
  }

private:

  // sends out both transforms with a fresh timestamp every tick
  void broadcastTimer()
  {
    auto now = this->get_clock()->now();
    map_to_odom_.header.stamp  = now;
    odom_to_base_.header.stamp = now;

    tf_broadcaster_->sendTransform(map_to_odom_);
    tf_broadcaster_->sendTransform(odom_to_base_);
  }

  // runs every time I click a 2D Goal Pose in RViz
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Goal received -> x: %.2f, y: %.2f",
      msg->pose.position.x,
      msg->pose.position.y);

    // this is where I want base_link to end up, in the map frame
    tf2::Transform goal_in_map;
    goal_in_map.setOrigin(tf2::Vector3(
      msg->pose.position.x,
      msg->pose.position.y,
      0.0));
    tf2::Quaternion q_goal(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w);
    goal_in_map.setRotation(q_goal);

    // this is the fixed odom -> base_link offset I set up above with fixed offset
    tf2::Transform odom_to_base_tf;
    odom_to_base_tf.setOrigin(tf2::Vector3(
      odom_to_base_.transform.translation.x,
      odom_to_base_.transform.translation.y,
      odom_to_base_.transform.translation.z));
    tf2::Quaternion q_odom_base(
      odom_to_base_.transform.rotation.x,
      odom_to_base_.transform.rotation.y,
      odom_to_base_.transform.rotation.z,
      odom_to_base_.transform.rotation.w);
    odom_to_base_tf.setRotation(q_odom_base);

    // now work backwards to figure out what map -> odom needs to be
    // so that base_link still lands exactly on the goal using inverse formula
    tf2::Transform new_map_to_odom = goal_in_map * odom_to_base_tf.inverse();

    map_to_odom_.transform.translation.x = new_map_to_odom.getOrigin().x();
    map_to_odom_.transform.translation.y = new_map_to_odom.getOrigin().y();
    map_to_odom_.transform.translation.z = new_map_to_odom.getOrigin().z();
    tf2::Quaternion q_new = new_map_to_odom.getRotation();
    map_to_odom_.transform.rotation.x = q_new.x();
    map_to_odom_.transform.rotation.y = q_new.y();
    map_to_odom_.transform.rotation.z = q_new.z();
    map_to_odom_.transform.rotation.w = q_new.w();

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