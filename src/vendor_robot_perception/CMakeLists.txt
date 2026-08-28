cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(vendor_robot_perception)


# ============================================================
# 编译选项
# ============================================================

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# ============================================================
# ROS 2
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)


# ============================================================
# TF
# ============================================================

find_package(tf2 REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)


# ============================================================
# PCL
# ============================================================

find_package(pcl_conversions REQUIRED)

find_package(
  PCL REQUIRED COMPONENTS
  common
  filters
  segmentation
  search
)


# ============================================================
# MoveIt
# ============================================================

find_package(moveit_msgs REQUIRED)

find_package(shape_msgs REQUIRED)

find_package(
  moveit_ros_planning_interface
  REQUIRED
)


# ============================================================
# target_estimator
# ============================================================

add_executable(
  target_estimator
  src/target_estimator.cpp
)


ament_target_dependencies(
  target_estimator

  rclcpp

  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs

  pcl_conversions

  moveit_msgs
  shape_msgs
  moveit_ros_planning_interface
)


target_include_directories(
  target_estimator
  PRIVATE
  ${PCL_INCLUDE_DIRS}
)


target_link_libraries(
  target_estimator
  ${PCL_LIBRARIES}
)


target_compile_definitions(
  target_estimator
  PRIVATE
  ${PCL_DEFINITIONS}
)


# ============================================================
# 安装可执行程序
# ============================================================

install(
  TARGETS
  target_estimator

  DESTINATION
  lib/${PROJECT_NAME}
)


# ============================================================
# 安装配置和launch
# ============================================================

install(
  DIRECTORY
  config
  launch

  DESTINATION
  share/${PROJECT_NAME}
)


ament_package()
