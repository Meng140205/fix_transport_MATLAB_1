cmake_minimum_required(VERSION 3.8)
project(dualarm_mpc_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TorquePlan.msg"
  "msg/PlanCommit.msg"
  "msg/ArmStatus.msg"
  DEPENDENCIES builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
