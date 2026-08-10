#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "vendor_robot_driver_core/cdrtdeapi.h"

namespace vendor_robot_driver_core
{

enum class ConnectionState : uint8_t
{
  DISCONNECTED = 0,
  CONNECTING = 1,
  SYNCHRONIZING = 2,
  RUNNING = 3,
  DEGRADED = 4,
  RECONNECTING = 5,
  FAULT = 6,
  STOPPED = 7
};

struct RtdeConfig
{
  std::string robot_ip;
  std::string network_interface;
  int frequency_hz{250};
  int io_poll_rate_hz{10};
  int protocol_version{3};
  bool nack{true};
  double servoj_cutoff_rad_s{50.0};
  double stop_deceleration_deg_s2{30.0};
  std::chrono::milliseconds command_watchdog{100};
  std::chrono::milliseconds reconnect_initial{250};
  std::chrono::milliseconds reconnect_max{5000};
  uint32_t max_consecutive_errors{3};
  std::chrono::milliseconds shutdown_timeout{3000};
};

struct JointState
{
  std::array<double, 6> position_rad{};
  std::array<double, 6> velocity_rad_s{};
  std::array<double, 6> acceleration_rad_s2{};
  std::chrono::steady_clock::time_point stamp{};
  uint64_t sequence{0};
  bool valid{false};
};

struct IoState
{
  uint64_t digital_inputs{0};
  uint64_t digital_outputs{0};
  std::chrono::steady_clock::time_point stamp{};
  bool valid{false};
};

struct DriverStatistics
{
  ConnectionState state{ConnectionState::DISCONNECTED};
  uint32_t consecutive_read_errors{0};
  uint32_t consecutive_write_errors{0};
  uint64_t reconnect_count{0};
  uint64_t cycle_count{0};
  uint64_t overrun_count{0};
  double last_cycle_ms{0.0};
  double max_cycle_ms{0.0};
  int last_rtde_error{0};
  std::string last_error_message;
  bool command_fresh{false};
  bool servo_skipped{false};
  uint64_t servoj_call_count{0};
};

enum class IoDomain : uint8_t { STANDARD = 0, CONFIGURABLE = 1, TOOL = 2 };

enum class SafeStopResult : uint8_t
{
  SUCCESS,
  ALREADY_STOPPED,
  NOT_CONNECTED,
  RETRYABLE_ERROR
};

class RtdeClient
{
public:
  explicit RtdeClient(RtdeConfig config);
  ~RtdeClient();
  RtdeClient(const RtdeClient &) = delete;
  RtdeClient & operator=(const RtdeClient &) = delete;

  bool start();
  void stop();
  void enable_servo(bool enabled);
  bool submit_joint_position(const std::array<double, 6> & position_rad);
  bool submit_speed_scaling(double scaling);
  bool submit_digital_output(IoDomain domain, uint8_t channel, bool value);
  void request_clear_error();
  void request_safe_stop();

  JointState joint_state() const;
  IoState io_state() const;
  DriverStatistics statistics() const;
  ConnectionState connection_state() const noexcept;
  bool wait_for_first_state(std::chrono::milliseconds timeout);

  const std::string & rtde_sdk_version() const {return rtde_sdk_version_;}
  static const char * state_name(ConnectionState state) noexcept;
  static std::string error_message(RtdeResult result);

private:
  struct Command
  {
    std::array<double, 6> position_rad{};
    std::chrono::steady_clock::time_point stamp{};
    uint64_t sequence{0};
    bool valid{false};
  };
  struct IoCommand { IoDomain domain; uint8_t channel; bool value; };

  void run();
  bool connect_and_synchronize();
  void disconnect();
  bool receive_state();
  bool receive_state_candidate(JointState & candidate);
  void commit_state(const JointState & candidate);
  void close_session(bool request_motion_stop);
  bool send_pending_commands();
  SafeStopResult try_safe_stop();
  void set_error(RtdeResult result, const char * context, bool write_error);
  void set_state(ConnectionState state);

  RtdeConfig config_;
  std::atomic<bool> running_{false};
  std::atomic<bool> worker_finished_{false};
  std::atomic<bool> servo_enabled_{false};
  // 停止请求序列号：每次 request_safe_stop() 递增。
  // worker 线程处理时对比 handled_sequence_，防止事件丢失。
  std::atomic<uint64_t> safe_stop_request_sequence_{0};
  std::atomic<std::chrono::steady_clock::time_point> servo_enable_stamp_{};
  std::atomic<bool> clear_error_requested_{false};
  std::atomic<bool> command_watchdog_stop_latched_{false};
  std::atomic<ConnectionState> connection_state_{ConnectionState::DISCONNECTED};
  std::thread worker_;

  RtdeHandle handle_{-1};
  int output_recipe_id_{-1};
  int input_recipe_id_{-1};
  std::string rtde_sdk_version_;
  std::array<uint8_t, 1024> rx_buffer_{};

  // ---- 以下字段只由 RTDE worker 线程访问 ----
  uint64_t safe_stop_handled_sequence_{0};
  uint32_t safe_stop_retry_count_{0};
  bool stop_sent_{false};
  bool servoj_succeeded_in_session_{false};
  uint64_t session_generation_{0};

  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  JointState joint_state_;
  IoState io_state_;
  DriverStatistics stats_;

  mutable std::mutex command_mutex_;
  Command command_;
  double pending_speed_scaling_{1.0};
  bool speed_scaling_dirty_{false};
  std::deque<IoCommand> io_commands_;
  std::atomic<uint64_t> servoj_call_count_{0};
};

}  // namespace vendor_robot_driver_core
