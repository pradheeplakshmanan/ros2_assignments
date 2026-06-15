cmake_minimum_required(VERSION 3.8)
project(question_1_tf2_goal_pose)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tf2_goal_node src/tf2_goal_node.cpp)

ament_target_dependencies(tf2_goal_node
  rclcpp tf2 tf2_ros geometry_msgs)

install(TARGETS tf2_goal_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()