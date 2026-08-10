#pragma once

#include <memory>
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64.hpp"

namespace vendor_robot_controllers
{
using CallbackReturn = controller_interface::CallbackReturn;
class SpeedScalingBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
private:
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>> publisher_;
};
}  // namespace vendor_robot_controllers
