# ROS2 Assignments

# Question 1 - TF2 Frame Management and 2D Goal Pose

A ROS2 C++ node that manages a three-frame TF tree: map -> odom -> base_link.

- Broadcasts both transforms at 10 Hz
- Subscribes to /goal_pose from RViz 2D Goal Pose button
- Moves base_link to the goal by adjusting map -> odom transform
- odom -> base_link always stays identity

# Question 2 - Nav2 Controller Plugin

A Nav2 controller plugin that drives the robot along a planned path.

- Inherits from nav2_core::Controller and loaded by Nav2 via pluginlib
- Steers toward each waypoint using heading error
- Constant forward speed, proportional angular velocity
- Moves to the next waypoint when within 0.3m