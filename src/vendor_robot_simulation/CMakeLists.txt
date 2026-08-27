cmake_minimum_required(VERSION 3.8)
project(vendor_robot_simulation)
find_package(ament_cmake REQUIRED)
# 安装仿真配置、启动文件和 Gazebo 专用 URDF
install(
  DIRECTORY
  config
  launch
  urdf
  world
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/joint_state_filter.py
  scripts/gazebo_planning_scene_sync.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

#cpp
find_package(rclcpp REQUIRED)

find_package(
  moveit_ros_planning_interface
  REQUIRED
)

find_package(
  moveit_task_constructor_core
  REQUIRED
)

find_package(
  moveit_msgs
  REQUIRED
)

find_package(
  geometry_msgs
  REQUIRED
)

find_package(
  Eigen3
  REQUIRED
)


add_executable(
  mtc_task_demo
  src/mtc_task_demo.cpp
)


ament_target_dependencies(
  mtc_task_demo

  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
)


target_link_libraries(
  mtc_task_demo

  Eigen3::Eigen
)


install(
  TARGETS
  mtc_task_demo

  DESTINATION
  lib/${PROJECT_NAME}
)
ament_package()


