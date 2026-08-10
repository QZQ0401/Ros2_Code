#include "vendor_robot_controllers/speed_scaling_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vendor_robot_controllers
{
CallbackReturn SpeedScalingBroadcaster::on_init() {return CallbackReturn::SUCCESS;}
controller_interface::InterfaceConfiguration
SpeedScalingBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}
controller_interface::InterfaceConfiguration
SpeedScalingBroadcaster::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    {"speed_scaling/speed_scaling_factor"}};
}
CallbackReturn SpeedScalingBroadcaster::on_configure(const rclcpp_lifecycle::State &)
{
  auto raw = get_node()->create_publisher<std_msgs::msg::Float64>(
    "~/speed_scaling_factor", rclcpp::SystemDefaultsQoS());
  publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(raw);
  return CallbackReturn::SUCCESS;
}
CallbackReturn SpeedScalingBroadcaster::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}
CallbackReturn SpeedScalingBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}
controller_interface::return_type SpeedScalingBroadcaster::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (publisher_ && publisher_->trylock()) {
    publisher_->msg_.data = state_interfaces_[0].get_value();
    publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}
}  // namespace vendor_robot_controllers
PLUGINLIB_EXPORT_CLASS(
  vendor_robot_controllers::SpeedScalingBroadcaster,
  controller_interface::ControllerInterface)
