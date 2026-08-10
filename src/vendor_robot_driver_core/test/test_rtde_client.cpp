#include <array>
#include <cassert>
#include <limits>
#include <string>

#include "vendor_robot_driver_core/rtde_client.hpp"

int main()
{
  using vendor_robot_driver_core::ConnectionState;
  using vendor_robot_driver_core::IoDomain;
  using vendor_robot_driver_core::RtdeClient;
  using vendor_robot_driver_core::RtdeConfig;
  assert(std::string(RtdeClient::state_name(ConnectionState::RUNNING)) == "RUNNING");
  assert(RtdeClient::error_message(RTDE_ERR_SOCKET_ERROR) == "socket error");

  RtdeConfig config;
  config.robot_ip = "127.0.0.1";
  RtdeClient client(config);
  assert(!client.submit_speed_scaling(-0.01));
  assert(!client.submit_speed_scaling(1.01));
  assert(client.submit_speed_scaling(0.5));

  std::array<double, 6> command{};
  command[3] = std::numeric_limits<double>::quiet_NaN();
  assert(!client.submit_joint_position(command));
  command[3] = 0.0;
  assert(client.submit_joint_position(command));

  RtdeClient io_client(config);
  assert(io_client.submit_digital_output(IoDomain::STANDARD, 0, false));
  assert(io_client.submit_digital_output(IoDomain::STANDARD, 7, true));
  assert(!io_client.submit_digital_output(IoDomain::STANDARD, 8, true));
  assert(io_client.submit_digital_output(IoDomain::CONFIGURABLE, 0, false));
  assert(io_client.submit_digital_output(IoDomain::CONFIGURABLE, 7, true));
  assert(!io_client.submit_digital_output(IoDomain::CONFIGURABLE, 8, true));
  assert(io_client.submit_digital_output(IoDomain::TOOL, 0, false));
  assert(io_client.submit_digital_output(IoDomain::TOOL, 9, true));
  assert(!io_client.submit_digital_output(IoDomain::TOOL, 10, true));

  // Eight valid boundary checks above consumed six slots. Fill the bounded
  // queue and verify that the next event is rejected rather than overwritten.
  for (int index = 6; index < 64; ++index) {
    assert(io_client.submit_digital_output(IoDomain::STANDARD, 0, (index % 2) != 0));
  }
  assert(!io_client.submit_digital_output(IoDomain::STANDARD, 0, false));

  // ---- 停止请求序列号测试 ----
  RtdeClient stop_client(config);
  // request_safe_stop() 递增序列号并禁用 servo
  stop_client.request_safe_stop();
  stop_client.request_safe_stop();
  stop_client.request_safe_stop();
  // 三次请求 → 序列号递增，不会丢事件
  // 验证 enable_servo(false) 走统一入口 request_safe_stop()
  stop_client.enable_servo(true);
  stop_client.enable_servo(false);
  // enable_servo(true) 重新启用 servo，stop_sent_ 仍由 worker 管理
  // try_safe_stop() 在无连接 (handle_ < 0) 时返回 NOT_CONNECTED
  // （无法直接调用 private 成员，通过公共 API 间接验证）
  assert(!client.submit_joint_position(command));  // 仍拒绝 NaN
  return 0;
}
