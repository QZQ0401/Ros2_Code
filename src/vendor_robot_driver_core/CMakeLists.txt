cmake_minimum_required(VERSION 3.8)
project(vendor_robot_driver_core)
find_package(ament_cmake REQUIRED)
add_library(${PROJECT_NAME} SHARED src/rtde_client.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/libcdrtdeapi.so"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/libcdrtdebase.so")
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
  INSTALL_RPATH "$ORIGIN")
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(FILES lib/libcdrtdeapi.so lib/libcdrtdebase.so DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(rtde_client_unit_test test/test_rtde_client.cpp)
  target_compile_features(rtde_client_unit_test PUBLIC cxx_std_17)
  target_link_libraries(rtde_client_unit_test ${PROJECT_NAME})
  add_test(NAME rtde_client_unit_test COMMAND rtde_client_unit_test)

  # 故障注入测试直接编译核心源码并提供模拟 SDK，覆盖句柄泄漏、零帧同步、
  # 命令看门狗停止失败和跨会话停止请求等高严重度路径。
  add_executable(rtde_client_fault_injection_test
    test/rtde_client_fault_injection.cpp
    src/rtde_client.cpp)
  target_compile_features(rtde_client_fault_injection_test PUBLIC cxx_std_17)
  target_include_directories(rtde_client_fault_injection_test PRIVATE include)
  target_link_libraries(rtde_client_fault_injection_test PRIVATE pthread)
  add_test(NAME rtde_client_fault_injection_test COMMAND rtde_client_fault_injection_test)
endif()
ament_package()
