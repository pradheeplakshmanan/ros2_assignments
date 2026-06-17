cmake_minimum_required(VERSION 3.8)
project(question_2_nav2_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# ── Build as a SHARED library (required for pluginlib)
add_library(simple_controller SHARED
  src/simple_controller.cpp)

target_include_directories(simple_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(simple_controller
  rclcpp nav2_core nav2_costmap_2d nav2_util
  pluginlib geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs angles)

# ── Register the plugin.xml with pluginlib
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS simple_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include/)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_export_include_directories(include)
ament_export_libraries(simple_controller)
ament_package()