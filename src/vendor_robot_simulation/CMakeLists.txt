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
ament_package()
