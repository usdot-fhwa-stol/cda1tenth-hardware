cmake_minimum_required(VERSION 3.5)
project(geometry_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleGeometry.msg"
  DEPENDENCIES geometry_msgs
)

# Export the runtime dependency so downstream packages can find it
ament_export_dependencies(rosidl_default_runtime)

ament_package()
