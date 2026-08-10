#include "vendor_robot_hardware/rtde_system.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <csignal>
#include <limits>
#include <stdexcept>
#include <unordered_set>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vendor_robot_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
double parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name, double fallback)
{
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end() ? fallback : std::stod(iterator->second);
}

int integer_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name, int fallback)
{
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end() ? fallback : std::stoi(iterator->second);
}

bool boolean_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name, bool fallback)
{
  const auto iterator = info.hardware_parameters.find(name);
  if (iterator == info.hardware_parameters.end()) {
    return fallback;
  }
  std::string value = iterator->second;
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  throw std::invalid_argument(name + " must be true or false");
}
}  // namespace

CallbackReturn RtdeSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  // 进程级忽略 SIGPIPE。vendor RTDE 库可能用不带 MSG_NOSIGNAL 的 send()，
  // controller_manager RT 线程未屏蔽此信号，socket 断开时内核会直接杀进程。
  // SIG_IGN 让 send() 返回 EPIPE 错误码而不是触发信号。
  std::signal(SIGPIPE, SIG_IGN);
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (!validate_interfaces() || !load_parameters()) {
    return CallbackReturn::ERROR;
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  position_state_.fill(nan);
  velocity_state_.fill(nan);
  acceleration_state_.fill(nan);
  position_command_.fill(nan);
  last_position_command_.fill(nan);
  last_command_velocity_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

bool RtdeSystem::validate_interfaces() const
{
  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(logger_, "Exactly 6 joints are required, got %zu", info_.joints.size());
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & joint : info_.joints) {
    if (!names.insert(joint.name).second) {
      RCLCPP_ERROR(logger_, "Duplicate joint name: %s", joint.name.c_str());
      return false;
    }
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(logger_, "%s must expose only a position command interface", joint.name.c_str());
      return false;
    }
    std::unordered_set<std::string> states;
    for (const auto & interface : joint.state_interfaces) {
      states.insert(interface.name);
    }
    if (states != std::unordered_set<std::string>{"position", "velocity", "acceleration"}) {
      RCLCPP_ERROR(
        logger_, "%s must expose position, velocity and acceleration states", joint.name.c_str());
      return false;
    }
  }
  const auto speed = std::find_if(
    info_.gpios.begin(), info_.gpios.end(),
    [](const auto & component) {return component.name == "speed_scaling";});
  const auto gpio = std::find_if(
    info_.gpios.begin(), info_.gpios.end(),
    [](const auto & component) {return component.name == "gpio";});
  const auto driver = std::find_if(
    info_.sensors.begin(), info_.sensors.end(),
    [](const auto & component) {return component.name == "driver";});
  if (speed == info_.gpios.end() || gpio == info_.gpios.end() ||
    driver == info_.sensors.end())
  {
    RCLCPP_ERROR(logger_, "speed_scaling/gpio/driver components are required");
    return false;
  }
  if (speed->command_interfaces.size() != 1 || speed->state_interfaces.size() != 1 ||
    speed->command_interfaces[0].name != "speed_scaling_factor" ||
    speed->state_interfaces[0].name != "speed_scaling_factor" ||
    gpio->command_interfaces.size() != 52 || gpio->state_interfaces.size() != 52 ||
    driver->state_interfaces.size() != 14)
  {
    RCLCPP_ERROR(logger_, "Custom command/state interface count or name is invalid");
    return false;
  }
  return true;
}

bool RtdeSystem::load_parameters()
{
  try {
    const auto ip = info_.hardware_parameters.find("robot_ip");
    if (ip == info_.hardware_parameters.end() || ip->second.empty()) {
      throw std::invalid_argument("robot_ip is required");
    }
    config_.robot_ip = ip->second;
    const auto network = info_.hardware_parameters.find("network_interface");
    config_.network_interface =
      network == info_.hardware_parameters.end() ? "" : network->second;
    config_.frequency_hz = integer_parameter(info_, "rtde_frequency", 250);
    // 钳位到 [50, 250]：过低状态延迟大，过高控制柜处理不过来
    if (config_.frequency_hz < 50) {
      RCLCPP_WARN(logger_, "rtde_frequency %d clamped to minimum 50 Hz", config_.frequency_hz);
      config_.frequency_hz = 50;
    } else if (config_.frequency_hz > 250) {
      RCLCPP_WARN(logger_, "rtde_frequency %d clamped to maximum 250 Hz", config_.frequency_hz);
      config_.frequency_hz = 250;
    }
    config_.io_poll_rate_hz = integer_parameter(info_, "io_poll_rate_hz", 10);
    config_.protocol_version = integer_parameter(info_, "rtde_protocol_version", 3);
    // servoj 截止频率由 RTDE 频率计算：cutoff = 50 / frequency_hz
    const double servoj_cutoff_frequency = parameter(info_, "servoj_cutoff_frequency", 50.0);
    config_.servoj_cutoff_rad_s = servoj_cutoff_frequency * static_cast<double>(config_.frequency_hz) / 250.0;
    config_.stop_deceleration_deg_s2 = parameter(info_, "stop_deceleration_deg_s2", 30.0);
    config_.max_consecutive_errors = static_cast<uint32_t>(
      integer_parameter(info_, "max_consecutive_errors", 3));
    config_.command_watchdog = std::chrono::milliseconds(
      integer_parameter(info_, "command_watchdog_ms", 100));
    state_watchdog_ =
      std::chrono::milliseconds(integer_parameter(info_, "state_watchdog_ms", 200));
    max_stale_state_cycles_ = integer_parameter(info_, "max_stale_state_cycles", 5);
    recovery_stable_cycles_ = integer_parameter(info_, "recovery_stable_cycles", 10);
    initial_state_timeout_ =
      std::chrono::milliseconds(integer_parameter(info_, "initial_state_timeout_ms", 10000));
    max_position_error_rad_ = parameter(info_, "max_position_error_rad", 1.0);
    position_error_is_fatal_ =
      boolean_parameter(info_, "position_error_is_fatal", false);
    max_velocity_rad_s_ = parameter(info_, "max_velocity_rad_s", 6.5);
    max_acceleration_rad_s2_ = parameter(info_, "max_acceleration_rad_s2", 40.0);
    max_cycle_delta_rad_ = parameter(info_, "max_cycle_delta_rad", 0.05);
    use_rtde_io_ = boolean_parameter(info_, "use_rtde_io", false);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "Invalid hardware parameter: %s", exception.what());
    return false;
  }
  if (config_.frequency_hz <= 0 || config_.io_poll_rate_hz < 0 ||
    config_.io_poll_rate_hz > config_.frequency_hz ||
    config_.servoj_cutoff_rad_s <= 0.0 ||
    config_.command_watchdog.count() <= 0 || state_watchdog_.count() <= 0 ||
    max_position_error_rad_ <= 0.0 || max_velocity_rad_s_ <= 0.0 ||
    max_acceleration_rad_s2_ <= 0.0 || max_cycle_delta_rad_ <= 0.0 ||
    max_stale_state_cycles_ < 0 || recovery_stable_cycles_ <= 0)
  {
    RCLCPP_ERROR(logger_, "Timeouts, frequencies and motion limits must be positive");
    return false;
  }
  // 控制周期出现异常值时用它兜底，避免 4 ms 量级的差分噪声被放大
  nominal_period_s_ = 1.0 / static_cast<double>(config_.frequency_hz);
  return true;
}

CallbackReturn RtdeSystem::on_configure(const rclcpp_lifecycle::State &)
{
  client_ = std::make_unique<vendor_robot_driver_core::RtdeClient>(config_);
  if (!client_->start()) {
    RCLCPP_ERROR(logger_, "Failed to start the RTDE worker");
    client_.reset();
    return CallbackReturn::ERROR;
  }
  if (!client_->wait_for_first_state(initial_state_timeout_)) {
    const auto statistics = client_->statistics();
    const std::string error =
      statistics.last_error_message.empty() ? "no lower-level error reported" :
      statistics.last_error_message;
    RCLCPP_ERROR(
      logger_,
      "RTDE initial state timeout after %ld ms: ip=%s, interface=%s, "
      "protocol=%d, connection_state=%s, error_code=%d, error=%s",
      initial_state_timeout_.count(), config_.robot_ip.c_str(),
      config_.network_interface.empty() ? "<default>" : config_.network_interface.c_str(),
      config_.protocol_version,
      vendor_robot_driver_core::RtdeClient::state_name(statistics.state),
      statistics.last_rtde_error, error.c_str());
    if (client_) {
      client_->stop();
      client_.reset();
    }
    return CallbackReturn::ERROR;
  }
  {
    const std::string sdk_ver = client_->rtde_sdk_version();
    RCLCPP_INFO(
      logger_,
      "RTDE data channel connected (RTDE SDK %s, protocol v%d, %d Hz)",
      sdk_ver.c_str(),
      config_.protocol_version, config_.frequency_hz);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RtdeSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!client_) {
    return CallbackReturn::ERROR;
  }
  const auto snapshot = client_->joint_state();
  if (!snapshot.valid) {
    RCLCPP_ERROR(logger_, "Cannot activate without a valid synchronized state");
    return CallbackReturn::ERROR;
  }
  position_state_ = snapshot.position_rad;
  velocity_state_ = snapshot.velocity_rad_s;
  acceleration_state_ = snapshot.acceleration_rad_s2;
  position_command_ = position_state_;
  last_position_command_ = position_state_;
  last_command_velocity_.fill(0.0);
  standard_digital_out_command_ = standard_digital_out_state_;
  configurable_digital_out_command_ = configurable_digital_out_state_;
  tool_digital_out_command_ = tool_digital_out_state_;
  standard_digital_out_sequence_command_.fill(0.0);
  configurable_digital_out_sequence_command_.fill(0.0);
  tool_digital_out_sequence_command_.fill(0.0);
  last_standard_digital_out_sequence_.fill(0.0);
  last_configurable_digital_out_sequence_.fill(0.0);
  last_tool_digital_out_sequence_.fill(0.0);
  pending_standard_digital_out_.fill(-1);
  pending_configurable_digital_out_.fill(-1);
  pending_tool_digital_out_.fill(-1);
  activation_reconnect_count_ = client_->statistics().reconnect_count;
  observed_reconnect_count_ = activation_reconnect_count_;
  last_state_sequence_ = snapshot.sequence;
  has_last_valid_state_ = true;
  communication_fault_latched_ = false;
  motion_recovery_required_ = false;
  recovery_fresh_cycles_ = recovery_stable_cycles_;
  rejected_command_count_ = 0.0;
  safety_stop_latched_ = 0.0;
  motion_command_active_ = false;
  motion_start_prepared_ = false;
  first_write_ = true;
  stale_state_cycles_ = 0;
  active_ = true;
  client_->enable_servo(false);
  RCLCPP_INFO(logger_, "Activated with command synchronized to measured joint position");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RtdeSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  motion_command_active_ = false;
  motion_start_prepared_ = false;
  pending_standard_digital_out_.fill(-1);
  pending_configurable_digital_out_.fill(-1);
  pending_tool_digital_out_.fill(-1);
  if (client_) {
    client_->enable_servo(false);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RtdeSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (client_) {
    client_->stop();
    client_.reset();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn RtdeSystem::on_error(const rclcpp_lifecycle::State &)
{
  // 仅停servo并标记inactive。不销毁client，worker线程保留重连能力。
  // 网络瞬断时，worker可在恢复后自动重连；操作员通过 lifecycle 手动恢复。
  // client 的生命周期由 on_cleanup()（主动清理）或析构函数管理。
  if (client_) {
    client_->enable_servo(false);
  }
  active_ = false;
  motion_command_active_ = false;
  motion_start_prepared_ = false;
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RtdeSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> result;
  for (std::size_t index = 0; index < 6; ++index) {
    result.emplace_back(info_.joints[index].name, "position", &position_state_[index]);
    result.emplace_back(info_.joints[index].name, "velocity", &velocity_state_[index]);
    result.emplace_back(info_.joints[index].name, "acceleration", &acceleration_state_[index]);
  }
  result.emplace_back("speed_scaling", "speed_scaling_factor", &speed_scaling_state_);
  for (std::size_t index = 0; index < 8; ++index) {
    result.emplace_back("gpio", "standard_digital_input_" + std::to_string(index),
      &standard_digital_in_state_[index]);
    result.emplace_back("gpio", "configurable_digital_input_" + std::to_string(index),
      &configurable_digital_in_state_[index]);
    result.emplace_back("gpio", "standard_digital_output_" + std::to_string(index),
      &standard_digital_out_state_[index]);
    result.emplace_back("gpio", "configurable_digital_output_" + std::to_string(index),
      &configurable_digital_out_state_[index]);
  }
  for (std::size_t index = 0; index < 10; ++index) {
    result.emplace_back("gpio", "tool_digital_input_" + std::to_string(index),
      &tool_digital_in_state_[index]);
    result.emplace_back("gpio", "tool_digital_output_" + std::to_string(index),
      &tool_digital_out_state_[index]);
  }
  result.emplace_back("driver", "connection_state", &connection_state_);
  result.emplace_back("driver", "state_age_ms", &state_age_ms_);
  result.emplace_back("driver", "overrun_count", &overrun_count_);
  result.emplace_back("driver", "reconnect_count", &reconnect_count_);
  result.emplace_back("driver", "control_cycle_count", &control_cycle_count_);
  result.emplace_back("driver", "last_cycle_ms", &last_cycle_ms_);
  result.emplace_back("driver", "max_cycle_ms", &max_cycle_ms_);
  result.emplace_back("driver", "command_fresh", &command_fresh_);
  result.emplace_back("driver", "last_rtde_error", &last_rtde_error_);
  result.emplace_back("driver", "read_error_count", &read_error_count_);
  result.emplace_back("driver", "write_error_count", &write_error_count_);
  result.emplace_back("driver", "safety_stop_latched", &safety_stop_latched_);
  result.emplace_back("driver", "rejected_command_count", &rejected_command_count_);
  result.emplace_back("driver", "servo_skipped", &servo_skipped_);
  return result;
}

std::vector<hardware_interface::CommandInterface> RtdeSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> result;
  for (std::size_t index = 0; index < 6; ++index) {
    result.emplace_back(info_.joints[index].name, "position", &position_command_[index]);
  }
  result.emplace_back("speed_scaling", "speed_scaling_factor", &speed_scaling_command_);
  for (std::size_t index = 0; index < 8; ++index) {
    result.emplace_back("gpio", "standard_digital_output_" + std::to_string(index),
      &standard_digital_out_command_[index]);
    result.emplace_back("gpio", "standard_digital_output_" + std::to_string(index) +
      "_write_sequence", &standard_digital_out_sequence_command_[index]);
    result.emplace_back("gpio", "configurable_digital_output_" + std::to_string(index),
      &configurable_digital_out_command_[index]);
    result.emplace_back("gpio", "configurable_digital_output_" + std::to_string(index) +
      "_write_sequence", &configurable_digital_out_sequence_command_[index]);
  }
  for (std::size_t index = 0; index < 10; ++index) {
    result.emplace_back("gpio", "tool_digital_output_" + std::to_string(index),
      &tool_digital_out_command_[index]);
    result.emplace_back("gpio", "tool_digital_output_" + std::to_string(index) +
      "_write_sequence", &tool_digital_out_sequence_command_[index]);
  }
  return result;
}

return_type RtdeSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::size_t start_joint_count = 0;
  std::size_t stop_joint_count = 0;
  for (const auto & joint : info_.joints) {
    const std::string interface = joint.name + "/position";
    start_joint_count +=
      std::count(start_interfaces.begin(), start_interfaces.end(), interface);
    stop_joint_count +=
      std::count(stop_interfaces.begin(), stop_interfaces.end(), interface);
  }
  if ((start_joint_count != 0 && start_joint_count != info_.joints.size()) ||
    (stop_joint_count != 0 && stop_joint_count != info_.joints.size()))
  {
    RCLCPP_ERROR(
      logger_, "All six joint position interfaces must be switched as one group");
    return return_type::ERROR;
  }
  motion_start_prepared_ = false;
  if (start_joint_count == info_.joints.size()) {
    if (!active_ || !client_) {
      RCLCPP_ERROR(logger_, "Cannot start joint command mode while hardware is inactive");
      return return_type::ERROR;
    }
    const auto snapshot = client_->joint_state();
    const auto statistics = client_->statistics();
    const double state_age_ms = snapshot.valid ?
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - snapshot.stamp).count() :
      std::numeric_limits<double>::infinity();
    if (!snapshot.valid ||
      statistics.state != vendor_robot_driver_core::ConnectionState::RUNNING ||
      !std::isfinite(state_age_ms) || state_age_ms > state_watchdog_.count())
    {
      RCLCPP_ERROR(
        logger_, "Cannot start joint command mode: RTDE state is invalid or stale");
      return return_type::ERROR;
    }
    if (motion_recovery_required_ && recovery_fresh_cycles_ < recovery_stable_cycles_) {
      RCLCPP_ERROR(
        logger_, "Cannot start joint command mode: RTDE recovery is not stable (%d/%d cycles)",
        recovery_fresh_cycles_, recovery_stable_cycles_);
      return return_type::ERROR;
    }
    prepared_start_position_ = snapshot.position_rad;
    motion_start_prepared_ = true;
  }
  return return_type::OK;
}

return_type RtdeSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::size_t start_joint_count = 0;
  std::size_t stop_joint_count = 0;
  for (const auto & joint : info_.joints) {
    const std::string interface = joint.name + "/position";
    start_joint_count +=
      std::count(start_interfaces.begin(), start_interfaces.end(), interface);
    stop_joint_count +=
      std::count(stop_interfaces.begin(), stop_interfaces.end(), interface);
  }
  if (stop_joint_count == info_.joints.size()) {
    motion_command_active_ = false;
    if (client_) {
      client_->enable_servo(false);
    }
    RCLCPP_INFO(logger_, "Joint position command mode stopped");
  }
  if (start_joint_count == info_.joints.size()) {
    if (!motion_start_prepared_ || !client_) {
      return return_type::ERROR;
    }
    position_state_ = prepared_start_position_;
    position_command_ = prepared_start_position_;
    last_position_command_ = prepared_start_position_;
    last_command_velocity_.fill(0.0);
    first_write_ = true;
    motion_command_active_ = true;
    motion_start_prepared_ = false;
    safety_stop_latched_ = 0.0;
    communication_fault_latched_ = false;
    motion_recovery_required_ = false;
    recovery_fresh_cycles_ = recovery_stable_cycles_;
    client_->enable_servo(true);
    RCLCPP_INFO(logger_, "Joint position command mode started from measured position");
  }
  return return_type::OK;
}

return_type RtdeSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!client_) {
    return return_type::ERROR;
  }

  const auto snapshot = client_->joint_state();
  const auto statistics = client_->statistics();
  connection_state_ = static_cast<double>(statistics.state);
  overrun_count_ = static_cast<double>(statistics.overrun_count);
  reconnect_count_ = static_cast<double>(statistics.reconnect_count);
  control_cycle_count_ = static_cast<double>(statistics.cycle_count);
  last_cycle_ms_ = statistics.last_cycle_ms;
  max_cycle_ms_ = statistics.max_cycle_ms;
  command_fresh_ = (!motion_recovery_required_ && statistics.command_fresh) ? 1.0 : 0.0;
  servo_skipped_ = statistics.servo_skipped ? 1.0 : 0.0;
  last_rtde_error_ = static_cast<double>(statistics.last_rtde_error);
  read_error_count_ = static_cast<double>(statistics.consecutive_read_errors);
  write_error_count_ = static_cast<double>(statistics.consecutive_write_errors);

  if (statistics.reconnect_count != observed_reconnect_count_) {
    handle_rtde_session_change(statistics.reconnect_count);
  }

  if (!snapshot.valid) {
    state_age_ms_ = -1.0;
    return handle_stale_state("state snapshot invalid");
  }

  state_age_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - snapshot.stamp).count();
  if (!std::isfinite(state_age_ms_) || state_age_ms_ > state_watchdog_.count()) {
    return handle_stale_state("state snapshot too old");
  }

  // ros2_control 与 RTDE 同频但不同相位，单个控制周期重复读取同一序列是正常的。
  // 超龄仍使用源时间戳判断；恢复稳定计数只统计真正前进的 RTDE 帧。
  const bool state_advanced = snapshot.sequence != last_state_sequence_;
  if (state_advanced) {
    last_state_sequence_ = snapshot.sequence;
  }

  stale_state_cycles_ = 0;
  position_state_ = snapshot.position_rad;
  velocity_state_ = snapshot.velocity_rad_s;
  acceleration_state_ = snapshot.acceleration_rad_s2;
  has_last_valid_state_ = true;

  if (motion_recovery_required_) {
    if (state_advanced) {
      const int previous_recovery_cycles = recovery_fresh_cycles_;
      recovery_fresh_cycles_ = std::min(
        recovery_fresh_cycles_ + 1, recovery_stable_cycles_);
      if (previous_recovery_cycles < recovery_stable_cycles_ &&
        recovery_fresh_cycles_ == recovery_stable_cycles_)
      {
        RCLCPP_WARN(
          logger_,
          "RTDE state has recovered for %d consecutive fresh frames. Measured joint state is valid; "
          "the motion controller must be reactivated to resume from the measured position.",
          recovery_stable_cycles_);
      }
    }
  } else {
    recovery_fresh_cycles_ = recovery_stable_cycles_;
  }

  const auto io = client_->io_state();
  if (io.valid) {
    for (std::size_t index = 0; index < 8; ++index) {
      standard_digital_in_state_[index] = (io.digital_inputs >> index) & 1U;
      configurable_digital_in_state_[index] = (io.digital_inputs >> (index + 8)) & 1U;
      standard_digital_out_state_[index] = (io.digital_outputs >> index) & 1U;
      configurable_digital_out_state_[index] = (io.digital_outputs >> (index + 8)) & 1U;
    }
    for (std::size_t index = 0; index < 10; ++index) {
      tool_digital_in_state_[index] = (io.digital_inputs >> (index + 16)) & 1U;
      tool_digital_out_state_[index] = (io.digital_outputs >> (index + 16)) & 1U;
    }
  }
  return return_type::OK;
}

return_type RtdeSystem::handle_stale_state(const char * reason)
{
  ++stale_state_cycles_;
  command_fresh_ = 0.0;
  servo_skipped_ = 1.0;

  if (!motion_recovery_required_) {
    motion_recovery_required_ = true;
    recovery_fresh_cycles_ = 0;
    latch_motion_fault(reason);
  }

  const bool threshold_reached =
    max_stale_state_cycles_ == 0 || stale_state_cycles_ >= max_stale_state_cycles_;
  if (threshold_reached && !communication_fault_latched_) {
    communication_fault_latched_ = true;
    RCLCPP_ERROR(
      logger_,
      "RTDE state remained stale for %d cycles; communication recovery is latched. "
      "The hardware component stays active so the RTDE worker can reconnect.",
      stale_state_cycles_);
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 1000,
      "RTDE state invalid: %s, stale cycles %d/%d",
      reason, stale_state_cycles_, max_stale_state_cycles_);
  }

  // 通信故障可恢复：不把整个 hardware component 打入 ERROR，保留 worker 重连能力。
  return return_type::OK;
}

void RtdeSystem::handle_rtde_session_change(uint64_t reconnect_count)
{
  observed_reconnect_count_ = reconnect_count;
  stale_state_cycles_ = 0;
  recovery_fresh_cycles_ = 0;
  motion_recovery_required_ = true;
  communication_fault_latched_ = true;
  latch_motion_fault("RTDE session changed");
  RCLCPP_ERROR(
    logger_,
    "RTDE session changed. Motion output is disabled while the hardware remains active for "
    "automatic reconnection; the controller must restart from the recovered measured state.");
}

bool RtdeSystem::limit_and_sanitize_command(const rclcpp::Duration & period)
{
  // 控制周期兜底：坏 dt 的危害是二阶的 —— 它会算出错误的 last_command_velocity_，
  // 让后续周期被加速度限幅误伤。
  double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 10.0 * nominal_period_s_) {
    dt = nominal_period_s_;
  }
  bool limited_any = false;
  double worst_position_error = 0.0;
  std::size_t worst_index = 0;
  for (std::size_t index = 0; index < 6; ++index) {
    // 实测位置非有限才是真正的致命错误
    if (!std::isfinite(position_state_[index])) {
      RCLCPP_ERROR(
        logger_, "Joint %zu measured position is non-finite", index + 1);
      return false;
    }
    // 上层还没写入命令：保持上一拍命令，不判故障
    if (!std::isfinite(position_command_[index])) {
      if (!std::isfinite(last_position_command_[index])) {
        RCLCPP_ERROR(
          logger_, "Joint %zu has no valid command and no previous command",
          index + 1);
        return false;
      }
      position_command_[index] = last_position_command_[index];
      continue;
    }
    if (!std::isfinite(last_position_command_[index])) {
      last_position_command_[index] = position_state_[index];
    }

    const double position_error = position_command_[index] - position_state_[index];
    if (std::abs(position_error) > std::abs(worst_position_error)) {
      worst_position_error = position_error;
      worst_index = index;
    }

    // 首拍以实测关节速度为基准，避免"从静止起步"算出虚假加速度
    // （运动过程中重新使能时尤其重要）
    const double previous_velocity = first_write_ ?
      (std::isfinite(velocity_state_[index]) ? velocity_state_[index] : 0.0) :
      last_command_velocity_[index];
    const double desired_delta =
      position_command_[index] - last_position_command_[index];
    const double desired_velocity = desired_delta / dt;

    // 1) 加速度限幅：本拍速度只能落在上一拍速度 ± a_max*dt 之内
    double velocity_low = previous_velocity - max_acceleration_rad_s2_ * dt;
    double velocity_high = previous_velocity + max_acceleration_rad_s2_ * dt;
    // 2) 速度限幅
    velocity_low = std::max(velocity_low, -max_velocity_rad_s_);
    velocity_high = std::min(velocity_high, max_velocity_rad_s_);
    if (velocity_low > velocity_high) {
      // 上一拍速度本身已超出速度上限（例如刚从外部运动接管），直接夹到速度区间
      velocity_low = -max_velocity_rad_s_;
      velocity_high = max_velocity_rad_s_;
    }
    const double limited_velocity =
      std::clamp(desired_velocity, velocity_low, velocity_high);
    // 3) 单周期位移硬上限，只用来防跳变
    const double limited_delta = std::clamp(
      limited_velocity * dt, -max_cycle_delta_rad_, max_cycle_delta_rad_);
    if (std::abs(limited_delta - desired_delta) > 1e-9) {
      limited_any = true;
    }
    position_command_[index] = last_position_command_[index] + limited_delta;
    last_command_velocity_[index] = limited_delta / dt;
  }

  if (limited_any) {
    rejected_command_count_ += 1.0;
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "Joint command was saturated (v<=%.2f rad/s, a<=%.1f rad/s^2, "
      "delta<=%.3f rad). Raise these limits or lower the MoveIt velocity scaling "
      "if this keeps firing.",
      max_velocity_rad_s_, max_acceleration_rad_s2_, max_cycle_delta_rad_);
  }
  // 跟随误差只做监控。servoj 是透传 + 低通，命令天然领先实测，
  // 把它当故障判据必然误触发；真正的跟随保护交给机器人控制器和 JTC 容差。
  if (std::abs(worst_position_error) > max_position_error_rad_) {
    if (position_error_is_fatal_) {
      RCLCPP_ERROR(
        logger_, "Joint %zu following error %f exceeds limit %f rad",
        worst_index + 1, worst_position_error, max_position_error_rad_);
      return false;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "Joint %zu following error %f rad exceeds %f rad; consider raising "
      "servoj_cutoff_frequency or lowering the MoveIt velocity scaling",
      worst_index + 1, worst_position_error, max_position_error_rad_);
  }
  return true;
}

return_type RtdeSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!active_ || !client_) {
    return return_type::OK;
  }
  if (motion_command_active_) {
    // 限幅整形失败只代表命令/状态出现 NaN/Inf，那才是真正的致命情况
    if (!limit_and_sanitize_command(period)) {
      rejected_command_count_ += 1.0;
      latch_motion_fault("non-finite joint command or state");
      return return_type::OK;
    }
    // last_command_velocity_ 已在限幅函数内按实际下发量更新
    last_position_command_ = position_command_;
    first_write_ = false;
    if (!client_->submit_joint_position(position_command_)) {
      rejected_command_count_ += 1.0;
      latch_motion_fault("command handoff failed");
      return return_type::OK;
    }
  }
  if (speed_scaling_command_ != last_speed_scaling_command_) {
    if (!client_->submit_speed_scaling(speed_scaling_command_)) {
      speed_scaling_command_ = last_speed_scaling_command_;
      rejected_command_count_ += 1.0;
      latch_motion_fault("invalid speed scaling");
      return return_type::OK;
    }
    last_speed_scaling_command_ = speed_scaling_command_;
    speed_scaling_state_ = speed_scaling_command_;
  }
  // RTDE IO 路径（默认关闭）。cd_rtde_*digitalout_set 是同步阻塞调用，
  // 与 250 Hz 状态流共用同一 RTDE socket 和线程。如果这些调用耗时较长，
  // 会推迟 receive_state()，导致 state_age 超标并触发运动停用与通信恢复锁存。
  // 硬件组件保持 active，以便 RTDE worker 继续自动重连。
  // 推荐通过 SDK 管理通道设置 IO：ros2 service call /sdk_manager/set_sdk_io
  if (use_rtde_io_) {
    auto capture_io_request =
      [this](
        double command, double sequence, double & last_sequence,
        int8_t & pending, const char * domain, std::size_t channel)
      {
        if (sequence == last_sequence) {
          return;
        }
        if (!std::isfinite(sequence) || sequence < 0.0 ||
          !std::isfinite(command) || (command != 0.0 && command != 1.0))
        {
          rejected_command_count_ += 1.0;
          RCLCPP_ERROR(
            logger_,
            "Rejected invalid %s digital output %zu command: value=%f sequence=%f",
            domain, channel, command, sequence);
        } else {
          pending = command >= 0.5 ? 1 : 0;
        }
        last_sequence = sequence;
      };

    for (std::size_t index = 0; index < 8; ++index) {
      capture_io_request(
        standard_digital_out_command_[index],
        standard_digital_out_sequence_command_[index],
        last_standard_digital_out_sequence_[index],
        pending_standard_digital_out_[index], "standard", index);
      capture_io_request(
        configurable_digital_out_command_[index],
        configurable_digital_out_sequence_command_[index],
        last_configurable_digital_out_sequence_[index],
        pending_configurable_digital_out_[index], "configurable", index);
    }
    for (std::size_t index = 0; index < 10; ++index) {
      capture_io_request(
        tool_digital_out_command_[index],
        tool_digital_out_sequence_command_[index],
        last_tool_digital_out_sequence_[index],
        pending_tool_digital_out_[index], "tool", index);
    }

    for (std::size_t index = 0; index < 8; ++index) {
      if (pending_standard_digital_out_[index] >= 0 &&
        client_->submit_digital_output(
          vendor_robot_driver_core::IoDomain::STANDARD, index,
          pending_standard_digital_out_[index] != 0))
      {
        pending_standard_digital_out_[index] = -1;
      }
      if (pending_configurable_digital_out_[index] >= 0 &&
        client_->submit_digital_output(
          vendor_robot_driver_core::IoDomain::CONFIGURABLE, index,
          pending_configurable_digital_out_[index] != 0))
      {
        pending_configurable_digital_out_[index] = -1;
      }
    }
    for (std::size_t index = 0; index < 10; ++index) {
      if (pending_tool_digital_out_[index] >= 0 &&
        client_->submit_digital_output(
          vendor_robot_driver_core::IoDomain::TOOL, index,
          pending_tool_digital_out_[index] != 0))
      {
        pending_tool_digital_out_[index] = -1;
      }
    }
  }
  return return_type::OK;
}

void RtdeSystem::latch_motion_fault(const char * reason)
{
  motion_command_active_ = false;
  motion_start_prepared_ = false;
  safety_stop_latched_ = 1.0;
  if (client_) {
    // enable_servo(false) 已通过统一入口产生一次安全停止请求，避免重复递增序列号。
    client_->enable_servo(false);
  }
  RCLCPP_ERROR(
    logger_,
    "Motion safety fault latched: %s. RTDE state/diagnostics remain available; "
    "deactivate and reactivate the motion controller to resynchronize from measured position",
    reason);
}

void RtdeSystem::safe_stop(const char * reason)
{
  if (client_) {
    client_->request_safe_stop();
  }
  RCLCPP_ERROR(logger_, "Safety stop requested: %s", reason);
}

}  // namespace vendor_robot_hardware

PLUGINLIB_EXPORT_CLASS(
  vendor_robot_hardware::RtdeSystem, hardware_interface::SystemInterface)
