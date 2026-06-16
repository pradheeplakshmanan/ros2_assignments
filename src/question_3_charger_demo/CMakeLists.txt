cmake_minimum_required(VERSION 3.8)
project(question_3_charger_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(question_3_charger_interfaces REQUIRED)

set(deps rclcpp rclcpp_action question_3_charger_interfaces)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server ${deps})

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client ${deps})

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server ${deps})

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client ${deps})

install(TARGETS
  service_server
  service_client
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()