cmake_minimum_required(VERSION 3.8)
project(question_3_charger_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate both the service and action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToCharger.srv"
  "action/GoToCharger.action"
)

ament_package()