#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "vendor_robot_driver_core/rtde_client.hpp"

namespace vendor_robot_hardware
{

class RtdeSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool validate_interfaces() const;
  bool load_parameters();
  // 旧的 validate_and_limit() 把跟随误差 / 差分速度 / 差分加速度超限一律当作
  // 致命错误并锁存运动故障。servoj 是透传 + 低通，命令永远领先实测，而且
  // 逐周期两次差分在非实时内核上噪声极大 —— 这些都不能当故障判据。
  // 现在改成限幅整形：只有 NaN/Inf 才致命，其余一律 saturation。
  bool limit_and_sanitize_command(const rclcpp::Duration & period);
  void latch_motion_fault(const char * reason);
  hardware_interface::return_type handle_stale_state(const char * reason);
  void handle_rtde_session_change(uint64_t reconnect_count);
  void safe_stop(const char * reason);

  rclcpp::Logger logger_{rclcpp::get_logger("vendor_robot_hardware.RtdeSystem")};
  vendor_robot_driver_core::RtdeConfig config_;
  std::unique_ptr<vendor_robot_driver_core::RtdeClient> client_;

  std::array<double, 6> position_state_{};
  std::array<double, 6> velocity_state_{};
  std::array<double, 6> acceleration_state_{};
  std::array<double, 6> position_command_{};
  std::array<double, 6> last_position_command_{};
  std::array<double, 6> last_command_velocity_{};

  double speed_scaling_state_{1.0};
  double speed_scaling_command_{1.0};
  double last_speed_scaling_command_{1.0};
  std::array<double, 8> standard_digital_out_command_{};
  std::array<double, 8> configurable_digital_out_command_{};
  std::array<double, 10> tool_digital_out_command_{};
  std::array<double, 8> standard_digital_out_sequence_command_{};
  std::array<double, 8> configurable_digital_out_sequence_command_{};
  std::array<double, 10> tool_digital_out_sequence_command_{};
  std::array<double, 8> last_standard_digital_out_sequence_{};
  std::array<double, 8> last_configurable_digital_out_sequence_{};
  std::array<double, 10> last_tool_digital_out_sequence_{};
  std::array<int8_t, 8> pending_standard_digital_out_{};
  std::array<int8_t, 8> pending_configurable_digital_out_{};
  std::array<int8_t, 10> pending_tool_digital_out_{};
  std::array<double, 8> standard_digital_out_state_{};
  std::array<double, 8> configurable_digital_out_state_{};
  std::array<double, 10> tool_digital_out_state_{};
  std::array<double, 8> standard_digital_in_state_{};
  std::array<double, 8> configurable_digital_in_state_{};
  std::array<double, 10> tool_digital_in_state_{};
  double connection_state_{0.0};
  double state_age_ms_{0.0};
  double overrun_count_{0.0};
  double reconnect_count_{0.0};
  double control_cycle_count_{0.0};
  double last_cycle_ms_{0.0};
  double max_cycle_ms_{0.0};
  double command_fresh_{0.0};
  double last_rtde_error_{0.0};
  double read_error_count_{0.0};
  double write_error_count_{0.0};
  double safety_stop_latched_{0.0};
  double rejected_command_count_{0.0};
  double servo_skipped_{0.0};

  bool active_{false};
  bool motion_command_active_{false};
  bool motion_start_prepared_{false};
  bool first_write_{true};
  std::array<double, 6> prepared_start_position_{};
  uint64_t activation_reconnect_count_{0};
  uint64_t observed_reconnect_count_{0};
  uint64_t last_state_sequence_{0};
  bool communication_fault_latched_{false};
  bool motion_recovery_required_{false};
  bool has_last_valid_state_{false};
  int recovery_fresh_cycles_{0};
  int recovery_stable_cycles_{10};
  // 限幅门限。默认值与 URDF 关节限制对齐（joint1~3 3.65 rad/s，joint4~6 6.28 rad/s），
  // 旧默认值 2.0 rad/s 比 URDF 自己还严，位移稍大就必然触发。
  double max_position_error_rad_{1.0};
  bool position_error_is_fatal_{false};
  double max_velocity_rad_s_{6.5};
  double max_acceleration_rad_s2_{40.0};
  double max_cycle_delta_rad_{0.05};
  bool use_rtde_io_{false};
  double nominal_period_s_{1.0 / 250.0};
  // read() 对瞬时无效 / 超龄状态的容忍周期数。单次抖动就把硬件打成 error
  // 会导致 ros2_control 停用全部控制器，正在执行的轨迹直接死，而且不会自动恢复。
  int max_stale_state_cycles_{250};
  int stale_state_cycles_{0};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::chrono::milliseconds state_watchdog_{200};
  std::chrono::milliseconds initial_state_timeout_{10000};
};

}  // namespace vendor_robot_hardware
