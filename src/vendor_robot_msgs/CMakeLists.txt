cmake_minimum_required(VERSION 3.8)
project(vendor_robot_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IOStates.msg"
  "msg/RobotMode.msg"
  "msg/SafetyMode.msg"
  "msg/DriverStatus.msg"
  "srv/SetIO.srv"
  "srv/SetSpeedScaling.srv"
  "srv/SetPayload.srv"
  "action/RecoverRobot.action"
  "action/MoveJ.action"
  "action/MoveL.action"
  "action/MoveC.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs trajectory_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
