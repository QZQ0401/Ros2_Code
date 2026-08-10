cmake_minimum_required(VERSION 3.8)
project(vendor_robot_hardware)
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(vendor_robot_driver_core REQUIRED)
add_library(${PROJECT_NAME} SHARED src/rtde_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle vendor_robot_driver_core)
pluginlib_export_plugin_description_file(hardware_interface vendor_robot_hardware.xml)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES vendor_robot_hardware.xml DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle vendor_robot_driver_core)
ament_package()
