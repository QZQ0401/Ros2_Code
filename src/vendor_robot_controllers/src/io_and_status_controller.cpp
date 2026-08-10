#include "vendor_robot_controllers/io_and_status_controller.hpp"

#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace vendor_robot_controllers
{
using controller_interface::interface_configuration_type;

namespace
{
constexpr std::size_t kExpectedCommandInterfaces = 53;
constexpr std::size_t kExpectedStateInterfaces = 52;

uint64_t valid_sequence_or_zero(double value)
{
  return std::isfinite(value) && value >= 0.0 ?
         static_cast<uint64_t>(value) : 0U;
}

uint64_t pack_output_command(bool value, uint64_t sequence)
{
  return (sequence << 1U) | static_cast<uint64_t>(value);
}

bool unpack_output_value(uint64_t packed)
{
  return (packed & 1U) != 0U;
}

uint64_t unpack_output_sequence(uint64_t packed)
{
  return packed >> 1U;
}

void update_output_command(std::atomic<uint64_t> & target, bool value)
{
  uint64_t current = target.load(std::memory_order_relaxed);
  while (true) {
    const uint64_t sequence = unpack_output_sequence(current) + 1U;
    const uint64_t desired = pack_output_command(value, sequence);
    if (target.compare_exchange_weak(
        current, desired, std::memory_order_release, std::memory_order_relaxed))
    {
      return;
    }
  }
}
}  // namespace

controller_interface::CallbackReturn IOAndStatusController::on_init()
{
  for (auto & value : standard_output_command_) {value.store(0);}
  for (auto & value : configurable_output_command_) {value.store(0);}
  for (auto & value : tool_output_command_) {value.store(0);}
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
IOAndStatusController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = interface_configuration_type::INDIVIDUAL;
  config.names.push_back("speed_scaling/speed_scaling_factor");
  for (std::size_t index = 0; index < 8; ++index) {
    config.names.push_back("gpio/standard_digital_output_" + std::to_string(index));
    config.names.push_back(
      "gpio/standard_digital_output_" + std::to_string(index) + "_write_sequence");
    config.names.push_back("gpio/configurable_digital_output_" + std::to_string(index));
    config.names.push_back(
      "gpio/configurable_digital_output_" + std::to_string(index) + "_write_sequence");
  }
  for (std::size_t index = 0; index < 10; ++index) {
    config.names.push_back("gpio/tool_digital_output_" + std::to_string(index));
    config.names.push_back(
      "gpio/tool_digital_output_" + std::to_string(index) + "_write_sequence");
  }
  return config;
}

controller_interface::InterfaceConfiguration
IOAndStatusController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = interface_configuration_type::INDIVIDUAL;
  for (std::size_t index = 0; index < 8; ++index) {
    config.names.push_back("gpio/standard_digital_input_" + std::to_string(index));
    config.names.push_back("gpio/configurable_digital_input_" + std::to_string(index));
    config.names.push_back("gpio/standard_digital_output_" + std::to_string(index));
    config.names.push_back("gpio/configurable_digital_output_" + std::to_string(index));
  }
  for (std::size_t index = 0; index < 10; ++index) {
    config.names.push_back("gpio/tool_digital_input_" + std::to_string(index));
    config.names.push_back("gpio/tool_digital_output_" + std::to_string(index));
  }
  return config;
}

controller_interface::CallbackReturn IOAndStatusController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  auto publisher = node->create_publisher<vendor_robot_msgs::msg::IOStates>(
    "~/io_states", rclcpp::SystemDefaultsQoS());
  io_publisher_ = std::make_shared<
    realtime_tools::RealtimePublisher<vendor_robot_msgs::msg::IOStates>>(publisher);
  set_io_service_ = node->create_service<vendor_robot_msgs::srv::SetIO>(
    "~/set_io", std::bind(
      &IOAndStatusController::set_io, this, std::placeholders::_1, std::placeholders::_2));
  set_speed_service_ = node->create_service<vendor_robot_msgs::srv::SetSpeedScaling>(
    "~/set_speed_scaling", std::bind(
      &IOAndStatusController::set_speed, this, std::placeholders::_1, std::placeholders::_2));
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IOAndStatusController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != kExpectedCommandInterfaces ||
    state_interfaces_.size() != kExpectedStateInterfaces)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command and %zu state interfaces, got %zu and %zu",
      kExpectedCommandInterfaces, kExpectedStateInterfaces,
      command_interfaces_.size(), state_interfaces_.size());
    return CallbackReturn::ERROR;
  }
  // Each GPIO uses a persistent value plus a monotonically increasing write sequence.
  speed_scaling_.store(command_interfaces_[0].get_value());
  std::size_t command_index = 1;
  for (std::size_t index = 0; index < 8; ++index) {
    const bool standard_value = command_interfaces_[command_index++].get_value() >= 0.5;
    const uint64_t standard_sequence =
      valid_sequence_or_zero(command_interfaces_[command_index++].get_value());
    standard_output_command_[index].store(
      pack_output_command(standard_value, standard_sequence), std::memory_order_release);

    const bool configurable_value = command_interfaces_[command_index++].get_value() >= 0.5;
    const uint64_t configurable_sequence =
      valid_sequence_or_zero(command_interfaces_[command_index++].get_value());
    configurable_output_command_[index].store(
      pack_output_command(configurable_value, configurable_sequence), std::memory_order_release);
  }
  for (std::size_t index = 0; index < 10; ++index) {
    const bool tool_value = command_interfaces_[command_index++].get_value() >= 0.5;
    const uint64_t tool_sequence =
      valid_sequence_or_zero(command_interfaces_[command_index++].get_value());
    tool_output_command_[index].store(
      pack_output_command(tool_value, tool_sequence), std::memory_order_release);
  }
  active_.store(true);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IOAndStatusController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type IOAndStatusController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  std::size_t command_index = 0;
  command_interfaces_[command_index++].set_value(speed_scaling_.load());
  auto publish_output_command =
    [this, &command_index](const std::atomic<uint64_t> & packed_command)
    {
      // value 与 sequence 来自同一个原子快照，实时线程不循环、不加锁。
      const uint64_t packed = packed_command.load(std::memory_order_acquire);
      command_interfaces_[command_index++].set_value(unpack_output_value(packed) ? 1.0 : 0.0);
      command_interfaces_[command_index++].set_value(
        static_cast<double>(unpack_output_sequence(packed)));
    };
  for (std::size_t index = 0; index < 8; ++index) {
    publish_output_command(standard_output_command_[index]);
    publish_output_command(configurable_output_command_[index]);
  }
  for (std::size_t index = 0; index < 10; ++index) {
    publish_output_command(tool_output_command_[index]);
  }

  if (io_publisher_ && io_publisher_->trylock()) {
    auto & message = io_publisher_->msg_;
    message.header.stamp = time;
    message.digital_inputs = 0;
    message.digital_outputs = 0;
    std::size_t state_index = 0;
    for (std::size_t index = 0; index < 8; ++index) {
      message.digital_inputs |=
        (static_cast<uint64_t>(state_interfaces_[state_index++].get_value() >= 0.5) << index);
      message.digital_inputs |=
        (static_cast<uint64_t>(state_interfaces_[state_index++].get_value() >= 0.5) << (index + 8));
      message.digital_outputs |=
        (static_cast<uint64_t>(state_interfaces_[state_index++].get_value() >= 0.5) << index);
      message.digital_outputs |=
        (static_cast<uint64_t>(state_interfaces_[state_index++].get_value() >= 0.5) << (index + 8));
    }
    for (std::size_t index = 0; index < 10; ++index) {
      message.digital_inputs |=
        (static_cast<uint64_t>(state_interfaces_[state_index++].get_value() >= 0.5) << (index + 16));
      message.digital_outputs |=
        (static_cast<uint64_t>(state_interfaces_[state_index++].get_value() >= 0.5) << (index + 16));
    }
    io_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

void IOAndStatusController::set_io(
  const std::shared_ptr<vendor_robot_msgs::srv::SetIO::Request> request,
  std::shared_ptr<vendor_robot_msgs::srv::SetIO::Response> response)
{
  if (!active_.load()) {
    response->success = false;
    response->error_code = -2;
    response->message = "IO controller is inactive";
    return;
  }
  bool valid = false;
  if (request->domain == request->STANDARD_DIGITAL_OUT && request->channel < 8) {
    update_output_command(standard_output_command_[request->channel], request->value);
    valid = true;
  } else if (
    request->domain == request->CONFIGURABLE_DIGITAL_OUT && request->channel < 8)
  {
    update_output_command(configurable_output_command_[request->channel], request->value);
    valid = true;
  } else if (request->domain == request->TOOL_DIGITAL_OUT && request->channel < 10) {
    update_output_command(tool_output_command_[request->channel], request->value);
    valid = true;
  }
  response->success = valid;
  response->error_code = valid ? 0 : -1;
  response->message = valid ?
    "queued for RTDE write; verify io_states feedback" :
    "invalid domain or channel";
}

void IOAndStatusController::set_speed(
  const std::shared_ptr<vendor_robot_msgs::srv::SetSpeedScaling::Request> request,
  std::shared_ptr<vendor_robot_msgs::srv::SetSpeedScaling::Response> response)
{
  const bool valid =
    std::isfinite(request->scaling) && request->scaling >= 0.0 && request->scaling <= 1.0;
  if (valid) {
    speed_scaling_.store(request->scaling);
  }
  response->success = valid;
  response->error_code = valid ? 0 : -1;
  response->message = valid ? "queued" : "scaling must be in [0, 1]";
}

}  // namespace vendor_robot_controllers

PLUGINLIB_EXPORT_CLASS(
  vendor_robot_controllers::IOAndStatusController, controller_interface::ControllerInterface)
