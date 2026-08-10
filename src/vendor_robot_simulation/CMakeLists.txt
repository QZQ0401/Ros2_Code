cmake_minimum_required(VERSION 3.8)
project(vendor_robot_simulation)
find_package(ament_cmake REQUIRED)
# 安装仿真配置、启动文件和 Gazebo 专用 URDF
install(
  DIRECTORY
    config
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
