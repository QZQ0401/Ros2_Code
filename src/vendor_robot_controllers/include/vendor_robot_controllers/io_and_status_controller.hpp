#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "vendor_robot_msgs/msg/io_states.hpp"
#include "vendor_robot_msgs/srv/set_io.hpp"
#include "vendor_robot_msgs/srv/set_speed_scaling.hpp"

namespace vendor_robot_controllers
{
using CallbackReturn = controller_interface::CallbackReturn;

class IOAndStatusController : public controller_interface::ControllerInterface
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
  void set_io(
    const std::shared_ptr<vendor_robot_msgs::srv::SetIO::Request> request,
    std::shared_ptr<vendor_robot_msgs::srv::SetIO::Response> response);
  void set_speed(
    const std::shared_ptr<vendor_robot_msgs::srv::SetSpeedScaling::Request> request,
    std::shared_ptr<vendor_robot_msgs::srv::SetSpeedScaling::Response> response);

  // 每个输出将 value(bit0) 与 sequence(bits1..63) 打包为一个原子值，
  // 实时 update() 单次 load 即可取得一致快照，不需要无界 seqlock 重试。
  std::array<std::atomic<uint64_t>, 8> standard_output_command_{};
  std::array<std::atomic<uint64_t>, 8> configurable_output_command_{};
  std::array<std::atomic<uint64_t>, 10> tool_output_command_{};
  std::atomic<double> speed_scaling_{1.0};
  std::atomic<bool> active_{false};
  rclcpp::Service<vendor_robot_msgs::srv::SetIO>::SharedPtr set_io_service_;
  rclcpp::Service<vendor_robot_msgs::srv::SetSpeedScaling>::SharedPtr set_speed_service_;
  std::shared_ptr<realtime_tools::RealtimePublisher<vendor_robot_msgs::msg::IOStates>>
    io_publisher_;
};

}  // namespace vendor_robot_controllers
