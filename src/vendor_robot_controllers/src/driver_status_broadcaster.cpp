#include "vendor_robot_controllers/driver_status_broadcaster.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "vendor_robot_driver_core/rtde_client.hpp"

namespace vendor_robot_controllers
{
controller_interface::CallbackReturn DriverStatusBroadcaster::on_init()
{
  return CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration
DriverStatusBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}
controller_interface::InterfaceConfiguration
DriverStatusBroadcaster::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    {"driver/connection_state", "driver/state_age_ms", "driver/overrun_count",
      "driver/reconnect_count", "driver/control_cycle_count", "driver/last_cycle_ms",
      "driver/max_cycle_ms", "driver/command_fresh", "driver/last_rtde_error",
      "driver/read_error_count", "driver/write_error_count",
      "driver/safety_stop_latched", "driver/rejected_command_count",
      "driver/servo_skipped"}};
}
controller_interface::CallbackReturn DriverStatusBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto raw = get_node()->create_publisher<vendor_robot_msgs::msg::DriverStatus>(
    "~/driver_status", rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_shared<
    realtime_tools::RealtimePublisher<vendor_robot_msgs::msg::DriverStatus>>(raw);
  return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn DriverStatusBroadcaster::on_activate(
  const rclcpp_lifecycle::State &) {return CallbackReturn::SUCCESS;}
controller_interface::CallbackReturn DriverStatusBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &) {return CallbackReturn::SUCCESS;}
controller_interface::return_type DriverStatusBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (publisher_ && publisher_->trylock()) {
    auto & message = publisher_->msg_;
    message.header.stamp = time;
    message.state = static_cast<uint8_t>(state_interfaces_[0].get_value());
    message.rtde_connected = message.state == message.RUNNING || message.state == message.DEGRADED;
    message.state_fresh = state_interfaces_[1].get_value() >= 0.0 &&
      state_interfaces_[1].get_value() < 100.0;
    message.overrun_count = static_cast<uint64_t>(state_interfaces_[2].get_value());
    message.reconnect_count = static_cast<uint64_t>(state_interfaces_[3].get_value());
    message.control_cycle_count = static_cast<uint64_t>(state_interfaces_[4].get_value());
    message.last_cycle_ms = state_interfaces_[5].get_value();
    message.max_cycle_ms = state_interfaces_[6].get_value();
    message.command_fresh = state_interfaces_[7].get_value() >= 0.5;
    message.last_rtde_error = static_cast<int32_t>(state_interfaces_[8].get_value());
    message.last_error_message =
      vendor_robot_driver_core::RtdeClient::error_message(
      static_cast<RtdeResult>(message.last_rtde_error));
    message.consecutive_read_errors = static_cast<uint32_t>(state_interfaces_[9].get_value());
    message.consecutive_write_errors = static_cast<uint32_t>(state_interfaces_[10].get_value());
    message.safety_stop_latched = state_interfaces_[11].get_value() >= 0.5;
    message.rejected_command_count =
      static_cast<uint32_t>(state_interfaces_[12].get_value());
    message.servo_skipped = state_interfaces_[13].get_value() >= 0.5;
    static const char * names[] = {
      "DISCONNECTED", "CONNECTING", "SYNCHRONIZING", "RUNNING",
      "DEGRADED", "RECONNECTING", "FAULT", "STOPPED"};
    message.state_name = message.state < 8 ? names[message.state] : "UNKNOWN";
    publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}
}  // namespace vendor_robot_controllers
PLUGINLIB_EXPORT_CLASS(
  vendor_robot_controllers::DriverStatusBroadcaster,
  controller_interface::ControllerInterface)
