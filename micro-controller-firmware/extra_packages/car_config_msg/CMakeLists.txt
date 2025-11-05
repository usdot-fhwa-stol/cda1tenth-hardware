cmake_minimum_required(VERSION 3.5)
project(car_config_msg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarConfig.msg"
)

# Export the runtime dependency so downstream packages can find it
ament_export_dependencies(rosidl_default_runtime)

ament_package()

