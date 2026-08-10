#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vendor_robot_msgs/msg/driver_status.hpp"

class DiagnosticsBridge : public rclcpp::Node
{
public:
  DiagnosticsBridge() : Node("driver_diagnostics")
  {
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    subscription_ = create_subscription<vendor_robot_msgs::msg::DriverStatus>(
      "driver_status_broadcaster/driver_status", 10,
      [this](const vendor_robot_msgs::msg::DriverStatus::SharedPtr message) {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = get_namespace() + std::string("/vendor_robot_driver");
        status.hardware_id = "G4";
        if (message->safety_stop_latched || !message->state_fresh ||
          message->state == message->FAULT ||
          message->state == message->DISCONNECTED)
        {
          status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        } else if (message->state != message->RUNNING ||
          message->consecutive_read_errors > 0 || message->consecutive_write_errors > 0)
        {
          status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        } else {
          status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        }
        status.message = message->safety_stop_latched ?
          "MOTION_SAFETY_STOP_LATCHED" : message->state_name;
        auto add = [&status](const std::string & key, const std::string & value) {
          diagnostic_msgs::msg::KeyValue item;
          item.key = key;
          item.value = value;
          status.values.push_back(item);
        };
        add("state_fresh", message->state_fresh ? "true" : "false");
        add("rtde_connected", message->rtde_connected ? "true" : "false");
        add("read_errors", std::to_string(message->consecutive_read_errors));
        add("write_errors", std::to_string(message->consecutive_write_errors));
        add("safety_stop_latched", message->safety_stop_latched ? "true" : "false");
        add("rejected_commands", std::to_string(message->rejected_command_count));
        add("overruns", std::to_string(message->overrun_count));
        add("last_rtde_error", std::to_string(message->last_rtde_error));
        add("last_error", message->last_error_message);
        array.status.push_back(status);
        publisher_->publish(array);
      });
  }
private:
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Subscription<vendor_robot_msgs::msg::DriverStatus>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiagnosticsBridge>());
  rclcpp::shutdown();
  return 0;
}
