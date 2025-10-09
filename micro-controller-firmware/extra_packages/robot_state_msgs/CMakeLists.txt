cmake_minimum_required(VERSION 3.5)
project(robot_state_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(robot_state_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  DEPENDENCIES robot_state_msgs
)

# Export the runtime dependency so downstream packages can find it
ament_export_dependencies(rosidl_default_runtime)

ament_package()