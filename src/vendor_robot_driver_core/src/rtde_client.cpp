#include "vendor_robot_driver_core/rtde_client.hpp"

#include <algorithm>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <pthread.h>
#include <utility>

namespace vendor_robot_driver_core
{
namespace
{
constexpr double kDegToRad = 0.017453292519943295769;
constexpr double kRadToDeg = 57.295779513082320877;
constexpr std::size_t kHeaderBytes = 4;
constexpr std::size_t kJointBytes = sizeof(double) * 6;

bool finite(const std::array<double, 6> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}
}  // namespace

RtdeClient::RtdeClient(RtdeConfig config) : config_(std::move(config)) {}
RtdeClient::~RtdeClient() {stop();}

bool RtdeClient::start()
{
  if (config_.robot_ip.empty() || config_.frequency_hz <= 0 || running_.exchange(true)) {
    return false;
  }
  worker_finished_.store(false, std::memory_order_release);
  worker_ = std::thread(&RtdeClient::run, this);
  return true;
}

void RtdeClient::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  servo_enabled_.store(false, std::memory_order_release);
  safe_stop_request_sequence_.fetch_add(1, std::memory_order_acq_rel);
  state_cv_.notify_all();
  if (!worker_.joinable()) {
    return;
  }

  std::unique_lock<std::mutex> lock(state_mutex_);
  const bool finished = state_cv_.wait_for(
    lock, config_.shutdown_timeout, [this] { return worker_finished_.load(); });
  lock.unlock();

  if (finished) {
    worker_.join();
    return;
  }
  // 超时：worker 线程已被闭源 SDK 永久阻塞。进程本身不可恢复 — 此时
  // controller_stopper 已将运动控制器停用，机器人控制器在 TCP 断开后会
  // 自动 protective-stop。让 OS 回收全部线程、socket 和 SDK 资源。
  std::fprintf(
    stderr,
    "\n[RtdeClient] worker did not finish within %lld ms — "
    "the RTDE receive call is unrecoverable. "
    "Terminating process so the OS reclaims all resources.\n\n",
    static_cast<long long>(config_.shutdown_timeout.count()));
  std::fflush(stderr);
  std::terminate();
}

void RtdeClient::enable_servo(bool enabled)
{
  if (!enabled) {
    request_safe_stop();
    return;
  }

  command_watchdog_stop_latched_.store(false, std::memory_order_release);
  servo_enabled_.store(true, std::memory_order_release);
  servo_enable_stamp_.store(std::chrono::steady_clock::now());
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_.valid = false;  // Never replay a command from an earlier activation.
}

bool RtdeClient::submit_joint_position(const std::array<double, 6> & position_rad)
{
  if (!finite(position_rad)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_.position_rad = position_rad;
  command_.stamp = std::chrono::steady_clock::now();
  command_.sequence++;
  command_.valid = true;
  return true;
}

bool RtdeClient::submit_speed_scaling(double scaling)
{
  if (!std::isfinite(scaling) || scaling < 0.0 || scaling > 1.0) {
    return false;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  pending_speed_scaling_ = scaling;
  speed_scaling_dirty_ = true;
  return true;
}

bool RtdeClient::submit_digital_output(IoDomain domain, uint8_t channel, bool value)
{
  const uint8_t limit = domain == IoDomain::TOOL ? 10 : 8;
  if (channel >= limit) {
    return false;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (io_commands_.size() >= 64) {
    return false;
  }
  io_commands_.push_back({domain, channel, value});
  return true;
}

void RtdeClient::request_clear_error() {clear_error_requested_.store(true);}

void RtdeClient::request_safe_stop()
{
  // 禁止后续继续发送 servoj，worker 下周期优先处理停止
  servo_enabled_.store(false, std::memory_order_release);
  // 序列号递增保证并发停止请求不被覆盖
  safe_stop_request_sequence_.fetch_add(1, std::memory_order_acq_rel);
}

JointState RtdeClient::joint_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return joint_state_;
}

IoState RtdeClient::io_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return io_state_;
}

DriverStatistics RtdeClient::statistics() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  DriverStatistics result = stats_;
  result.state = connection_state_.load();
  return result;
}

ConnectionState RtdeClient::connection_state() const noexcept {return connection_state_.load();}

bool RtdeClient::wait_for_first_state(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  return state_cv_.wait_for(
    lock, timeout,
    [this] {
      return joint_state_.valid &&
             connection_state_.load(std::memory_order_acquire) == ConnectionState::RUNNING;
    });
}

void RtdeClient::run()
{
  // The vendor RTDE binary may use send() without MSG_NOSIGNAL.  Block SIGPIPE
  // in the only thread that calls it so a closed controller socket becomes an
  // RTDE error/reconnect event instead of terminating ros2_control_node.
  sigset_t blocked_signals;
  sigemptyset(&blocked_signals);
  sigaddset(&blocked_signals, SIGPIPE);
  (void)pthread_sigmask(SIG_BLOCK, &blocked_signals, nullptr);

  auto backoff = config_.reconnect_initial;
  while (running_.load()) {
    if (!connect_and_synchronize()) {
      set_state(ConnectionState::RECONNECTING);
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_cv_.wait_for(lock, backoff, [this] {return !running_.load();});
      }
      backoff = std::min(backoff * 2, config_.reconnect_max);
      continue;
    }
    backoff = config_.reconnect_initial;
    set_state(ConnectionState::RUNNING);

    const auto period_us = std::chrono::microseconds(
      static_cast<int64_t>(1000000.0 / static_cast<double>(config_.frequency_hz)));
    while (running_.load() && connection_state_.load() == ConnectionState::RUNNING) {
      const auto cycle_deadline = std::chrono::steady_clock::now() + period_us;
      if (!receive_state()) {
        DriverStatistics snapshot = statistics();
        if (snapshot.consecutive_read_errors >= config_.max_consecutive_errors) {
          servo_enabled_.store(false);
          set_state(ConnectionState::RECONNECTING);
          break;
        }
        set_state(ConnectionState::DEGRADED);
      }
      if (!send_pending_commands()) {
        DriverStatistics snapshot = statistics();
        if (snapshot.consecutive_write_errors >= config_.max_consecutive_errors) {
          servo_enabled_.store(false);
          set_state(ConnectionState::RECONNECTING);
          break;
        }
      }
      const auto cycle_start = cycle_deadline - period_us;
      const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
      const double elapsed_ms =
        std::chrono::duration<double, std::milli>(elapsed).count();
      const double target_ms = 1000.0 / static_cast<double>(config_.frequency_hz);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stats_.cycle_count++;
        stats_.last_cycle_ms = elapsed_ms;
        stats_.max_cycle_ms = std::max(stats_.max_cycle_ms, elapsed_ms);
        if (elapsed_ms > target_ms * 1.2) {
          stats_.overrun_count++;
        }
      }
      if (connection_state_.load() == ConnectionState::DEGRADED) {
        set_state(ConnectionState::RUNNING);
      }
      // 速率控制：sleep_until 免疫信号中断
      std::this_thread::sleep_until(cycle_deadline);
    }
    disconnect();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stats_.reconnect_count++;
      joint_state_.valid = false;
      io_state_.valid = false;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_.valid = false;
      io_commands_.clear();
      speed_scaling_dirty_ = false;
    }
  }
  disconnect();
  set_state(ConnectionState::STOPPED);
  worker_finished_.store(true);
  state_cv_.notify_all();
}

bool RtdeClient::connect_and_synchronize()
{
  // 清理任何异常残留会话。建链失败路径不发送 servo_stop，避免在从未
  // 执行过 servoj 的会话上触发额外阻塞或“不允许执行”错误。
  close_session(false);
  set_state(ConnectionState::CONNECTING);

  uint8_t version[21] = {};
  RtdeResult result = cd_rtde_get_sdk_version(version);
  if (result != RTDE_ERR_NONE) {
    set_error(result, "get RTDE SDK version", false);
    return false;
  }
  rtde_sdk_version_ = std::string(reinterpret_cast<char *>(version));

  result = cd_rtde_create(
    &handle_, config_.robot_ip.c_str(),
    config_.network_interface.empty() ? nullptr : config_.network_interface.c_str(),
    nullptr, config_.protocol_version, config_.nack ? 1 : 0);
  if (result != RTDE_ERR_NONE || handle_ < 0) {
    set_error(result, "create connection", false);
    handle_ = -1;
    return false;
  }

  char output_vars[] =
    "actualJointPosition,actualJointVelocity,actualJointAcceleration";
  result = cd_rtde_output_setup(
    handle_, output_vars, config_.frequency_hz, &output_recipe_id_);
  if (result != RTDE_ERR_NONE) {
    set_error(result, "configure output recipe", false);
    close_session(false);
    return false;
  }

  result = cd_rtde_output_control(handle_, output_recipe_id_, 1);
  if (result != RTDE_ERR_NONE) {
    set_error(result, "start output stream", false);
    close_session(false);
    return false;
  }

  set_state(ConnectionState::SYNCHRONIZING);

  // 控制柜启动输出流后可能短暂返回全零占位帧。同步阶段只解析候选帧，
  // 不写公共 joint_state_，因此 wait_for_first_state() 不会被占位帧提前唤醒。
  constexpr auto kZeroFrameGrace = std::chrono::milliseconds(200);
  constexpr int kRequiredStableZeroFrames = 3;
  const auto sync_start = std::chrono::steady_clock::now();
  int stable_zero_frames = 0;
  JointState last_candidate{};

  while (running_.load(std::memory_order_acquire)) {
    JointState candidate{};
    if (!receive_state_candidate(candidate)) {
      close_session(false);
      return false;
    }

    const bool all_zero = std::all_of(
      candidate.position_rad.begin(), candidate.position_rad.end(),
      [](double value) {return std::abs(value) < 1e-12;});

    if (!all_zero) {
      commit_state(candidate);
      ++session_generation_;
      servoj_succeeded_in_session_ = false;
      stop_sent_ = true;
      safe_stop_retry_count_ = 0;
      // 旧会话的停止债务不能带入新会话。新会话默认 servo 已禁用，
      // 只有新的 command-mode activation 才能重新启用 servoj。
      safe_stop_handled_sequence_ =
        safe_stop_request_sequence_.load(std::memory_order_acquire);
      return true;
    }

    ++stable_zero_frames;
    last_candidate = candidate;
    if (std::chrono::steady_clock::now() - sync_start >= kZeroFrameGrace &&
      stable_zero_frames >= kRequiredStableZeroFrames)
    {
      // 机械臂确实可能处于全零位。经过时间窗口和连续帧确认后才提交。
      commit_state(last_candidate);
      ++session_generation_;
      servoj_succeeded_in_session_ = false;
      stop_sent_ = true;
      safe_stop_retry_count_ = 0;
      safe_stop_handled_sequence_ =
        safe_stop_request_sequence_.load(std::memory_order_acquire);
      return true;
    }
  }

  close_session(false);
  return false;
}

void RtdeClient::close_session(bool request_motion_stop)
{
  if (handle_ < 0) {
    output_recipe_id_ = -1;
    input_recipe_id_ = -1;
    stop_sent_ = true;
    servoj_succeeded_in_session_ = false;
    safe_stop_retry_count_ = 0;
    safe_stop_handled_sequence_ =
      safe_stop_request_sequence_.load(std::memory_order_acquire);
    return;
  }

  if (request_motion_stop && servoj_succeeded_in_session_) {
    (void)try_safe_stop();
  }
  if (output_recipe_id_ >= 0) {
    (void)cd_rtde_output_control(handle_, static_cast<uint32_t>(output_recipe_id_), 2);
  }
  (void)cd_rtde_destroy(handle_);

  handle_ = -1;
  output_recipe_id_ = -1;
  input_recipe_id_ = -1;
  stop_sent_ = true;
  servoj_succeeded_in_session_ = false;
  safe_stop_retry_count_ = 0;
  safe_stop_handled_sequence_ =
    safe_stop_request_sequence_.load(std::memory_order_acquire);
}

void RtdeClient::disconnect()
{
  close_session(true);
}

bool RtdeClient::receive_state_candidate(JointState & candidate)
{
  const RtdeResult result = cd_rtde_output_data_receive(
    handle_, static_cast<uint32_t>(output_recipe_id_), rx_buffer_.data(), rx_buffer_.size());
  if (result != RTDE_ERR_NONE) {
    set_error(result, "receive state", false);
    return false;
  }

  std::array<double, 6> position_deg{};
  std::array<double, 6> velocity_deg{};
  std::array<double, 6> acceleration_deg{};
  std::memcpy(position_deg.data(), rx_buffer_.data() + kHeaderBytes, kJointBytes);
  std::memcpy(velocity_deg.data(), rx_buffer_.data() + kHeaderBytes + kJointBytes, kJointBytes);
  std::memcpy(
    acceleration_deg.data(), rx_buffer_.data() + kHeaderBytes + 2 * kJointBytes, kJointBytes);

  if (!finite(position_deg) || !finite(velocity_deg) || !finite(acceleration_deg)) {
    set_error(RTDE_ERR_DATA_TYPE, "non-finite state", false);
    return false;
  }

  for (std::size_t index = 0; index < 6; ++index) {
    candidate.position_rad[index] = position_deg[index] * kDegToRad;
    candidate.velocity_rad_s[index] = velocity_deg[index] * kDegToRad;
    candidate.acceleration_rad_s2[index] = acceleration_deg[index] * kDegToRad;
  }
  candidate.stamp = std::chrono::steady_clock::now();
  candidate.valid = true;
  return true;
}

void RtdeClient::commit_state(const JointState & candidate)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    joint_state_.position_rad = candidate.position_rad;
    joint_state_.velocity_rad_s = candidate.velocity_rad_s;
    joint_state_.acceleration_rad_s2 = candidate.acceleration_rad_s2;
    joint_state_.stamp = candidate.stamp;
    ++joint_state_.sequence;
    joint_state_.valid = true;
    stats_.consecutive_read_errors = 0;
  }
  state_cv_.notify_all();
}

bool RtdeClient::receive_state()
{
  JointState candidate{};
  if (!receive_state_candidate(candidate)) {
    return false;
  }
  commit_state(candidate);

  uint64_t state_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_sequence = joint_state_.sequence;
  }

  const uint64_t io_poll_divider = config_.io_poll_rate_hz > 0 ?
    static_cast<uint64_t>(std::max(1, config_.frequency_hz / config_.io_poll_rate_hz)) : 0;
  if (io_poll_divider > 0 &&
    (state_sequence == 1 || state_sequence % io_poll_divider == 0))
  {
    uint64_t inputs = 0;
    uint64_t outputs = 0;
    if (cd_rtde_alldigitalinputbits_get(handle_, &inputs) == RTDE_ERR_NONE &&
      cd_rtde_alldigitaloutputbits_get(handle_, &outputs) == RTDE_ERR_NONE)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      io_state_ = {inputs, outputs, std::chrono::steady_clock::now(), true};
    }
  }
  return true;
}

bool RtdeClient::send_pending_commands()
{
  bool management_write_succeeded = false;

  // 停止具有最高优先级：未确认完成前禁止发送任何其他 RTDE 命令。
  const uint64_t requested_sequence =
    safe_stop_request_sequence_.load(std::memory_order_acquire);
  if (requested_sequence != safe_stop_handled_sequence_) {
    const SafeStopResult stop_result = try_safe_stop();
    if (stop_result == SafeStopResult::SUCCESS ||
        stop_result == SafeStopResult::ALREADY_STOPPED) {
      safe_stop_handled_sequence_ = requested_sequence;
    } else {
      return false;  // 停止未完成，禁止发送后续命令
    }
  }

  if (clear_error_requested_.exchange(false)) {
    const RtdeResult result = cd_rtde_reseterror(handle_);
    if (result != RTDE_ERR_NONE) {
      set_error(result, "clear error", true);
      return false;
    }
    management_write_succeeded = true;
  }

  Command command;
  double speed = 1.0;
  bool speed_dirty = false;
  IoCommand io{IoDomain::STANDARD, 0, false};
  bool io_pending = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command = command_;
    speed = pending_speed_scaling_;
    speed_dirty = speed_scaling_dirty_;
    if (!io_commands_.empty()) {
      io = io_commands_.front();
      io_pending = true;
    }
  }
  if (speed_dirty) {
    const RtdeResult result = cd_rtde_percentvelocity_set(handle_, speed * 100.0);
    if (result != RTDE_ERR_NONE) {
      set_error(result, "set speed scaling", true);
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (speed_scaling_dirty_ && pending_speed_scaling_ == speed) {
        speed_scaling_dirty_ = false;
      }
    }
    management_write_succeeded = true;
  }
  if (io_pending) {
    RtdeResult result = RTDE_ERR_PARAM;
    if (io.domain == IoDomain::STANDARD) {
      result = cd_rtde_standarddigitalout_set(handle_, io.channel, io.value ? 1 : 0);
    } else if (io.domain == IoDomain::CONFIGURABLE) {
      result = cd_rtde_configurabledigitalout_set(handle_, io.channel, io.value ? 1 : 0);
    } else {
      result = cd_rtde_tooldigitalout_set(handle_, io.channel, io.value ? 1 : 0);
    }
    if (result != RTDE_ERR_NONE) {
      set_error(result, "set digital output", true);
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!io_commands_.empty()) {
        io_commands_.pop_front();
      }
    }
    management_write_succeeded = true;
  }
  if (management_write_succeeded) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stats_.consecutive_write_errors = 0;
  }
  if (!servo_enabled_.load()) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stats_.command_fresh = false;
    stats_.servo_skipped = true;
    return true;
  }
  const auto now = std::chrono::steady_clock::now();
  const bool fresh = command.valid && now - command.stamp <= config_.command_watchdog;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stats_.command_fresh = fresh;
  }
  bool command_expired = false;
  if (!fresh) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      stats_.servo_skipped = true;
      const auto enable_stamp = servo_enable_stamp_.load();
      constexpr auto kServoGrace = std::chrono::milliseconds(200);
      if (enable_stamp.time_since_epoch().count() == 0 ||
        now - enable_stamp > kServoGrace)
      {
        command_expired = true;
      }
    }
    if (!command_expired) {
      return true;
    }
    // 命令看门狗触发后必须锁存关闭。新鲜命令本身不能重新启用 servoj，
    // 只有新的 controller command-mode activation 才能调用 enable_servo(true)。
    if (!command_watchdog_stop_latched_.exchange(true, std::memory_order_acq_rel)) {
      request_safe_stop();
    }
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stats_.servo_skipped = false;
  }

  std::array<double, 6> degrees{};
  for (std::size_t index = 0; index < 6; ++index) {
    degrees[index] = command.position_rad[index] * kRadToDeg;
  }

  // ===============诊断日志：每 2 秒打印一次 servoj 实际下发的目标位置（度）
  // {
  //   static auto last_print = std::chrono::steady_clock::now();
  //   const auto now = std::chrono::steady_clock::now();
  //   if (now - last_print >= std::chrono::seconds(2)) {
  //     last_print = now;
  //     std::fprintf(
  //       stderr,
  //       "[RtdeClient] servoj seq=%lu cmd_deg=[%.4f %.4f %.4f %.4f %.4f %.4f]\n",
  //       static_cast<unsigned long>(servoj_call_count_.load() + 1),
  //       degrees[0], degrees[1], degrees[2],
  //       degrees[3], degrees[4], degrees[5]);
  //     std::fflush(stderr);
  //   }
  // }

  const uint64_t call_nr = servoj_call_count_.fetch_add(1) + 1;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stats_.servoj_call_count = call_nr;
  }
  const RtdeResult result =
    cd_rtde_servoj(handle_, 1,degrees.data(), 0, nullptr, config_.servoj_cutoff_rad_s);
  if (result != RTDE_ERR_NONE) {
    set_error(result, "send servoj", true);
    return false;
  }
  // 机器人重新接受了运动命令，下一次停止必须真正下发
  servoj_succeeded_in_session_ = true;
  stop_sent_ = false;
  std::lock_guard<std::mutex> lock(state_mutex_);
  stats_.consecutive_write_errors = 0;
  return true;
}

SafeStopResult RtdeClient::try_safe_stop()
{
  if (stop_sent_ || !servoj_succeeded_in_session_) {
    // 当前会话从未成功执行过 servoj 时，停止是幂等完成状态，不能把旧会话
    // 的停止请求变成新会话的永久重试自锁。
    stop_sent_ = true;
    return SafeStopResult::ALREADY_STOPPED;
  }

  if (handle_ < 0) {
    return SafeStopResult::NOT_CONNECTED;
  }

  const RtdeResult result =
    cd_rtde_servo_stop(handle_, config_.stop_deceleration_deg_s2);

  if (result != RTDE_ERR_NONE) {
    set_error(result, "servo stop", true);
    // 失败时绝不能设置 stop_sent_，下周期重试
    ++safe_stop_retry_count_;
    return SafeStopResult::RETRYABLE_ERROR;
  }

  stop_sent_ = true;
  safe_stop_retry_count_ = 0;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stats_.command_fresh = false;
    stats_.servo_skipped = true;
  }

  return SafeStopResult::SUCCESS;
}

void RtdeClient::set_error(RtdeResult result, const char * context, bool write_error)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  stats_.last_rtde_error = static_cast<int>(result);
  stats_.last_error_message = std::string(context) + ": " + error_message(result);
  if (write_error) {
    stats_.consecutive_write_errors++;
  } else {
    stats_.consecutive_read_errors++;
  }
}

void RtdeClient::set_state(ConnectionState state)
{
  connection_state_.store(state, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stats_.state = state;
  }
  state_cv_.notify_all();
}

const char * RtdeClient::state_name(ConnectionState state) noexcept
{
  switch (state) {
    case ConnectionState::DISCONNECTED: return "DISCONNECTED";
    case ConnectionState::CONNECTING: return "CONNECTING";
    case ConnectionState::SYNCHRONIZING: return "SYNCHRONIZING";
    case ConnectionState::RUNNING: return "RUNNING";
    case ConnectionState::DEGRADED: return "DEGRADED";
    case ConnectionState::RECONNECTING: return "RECONNECTING";
    case ConnectionState::FAULT: return "FAULT";
    case ConnectionState::STOPPED: return "STOPPED";
  }
  return "UNKNOWN";
}

std::string RtdeClient::error_message(RtdeResult result)
{
  switch (result) {
    case RTDE_ERR_NONE: return "success";
    case RTDE_ERR_PACKAGE_TYPE: return "unrecognized packet type";
    case RTDE_ERR_PORT_UNMATCH: return "interface/port mismatch";
    case RTDE_ERR_PAYLOAD_LEN: return "invalid payload length";
    case RTDE_ERR_PROTOCOL_VER_NSUPPORT: return "unsupported protocol version";
    case RTDE_ERR_DATA_TYPE: return "invalid data type";
    case RTDE_ERR_PARAM: return "invalid parameter";
    case RTDE_ERR_EXECUTE_NALLOW: return "operation not allowed";
    case RTDE_ERR_IO_NSUPPORT: return "IO not supported";
    case RTDE_ERR_METHOD_INDEX: return "method not supported";
    case RTDE_ERR_VARIABLE_CODE: return "unknown recipe variable";
    case RTDE_ERR_VARIABLE_NSUPPORT: return "recipe variable not supported";
    case RTDE_ERR_RECIPE_LEN: return "recipe too long";
    case RTDE_ERR_RECIPE_INPUT_VAR_CODE_REUSED: return "recipe variable reused";
    case RTDE_ERR_RECIPE_INPUT_LIMIT: return "input recipe limit reached";
    case RTDE_ERR_ULTRALIMIT: return "value exceeds limit";
    case RTDE_ERR_METHOD_OVER: return "method rate limit exceeded";
    case RTDE_ERR_DATALEN_ULTRALIMIT: return "data length exceeds limit";
    case RTDE_ERR_DATA_NOT_EXIST: return "data does not exist";
    case RTDE_ERR_SOCKET_ERROR: return "socket error";
    case RTDE_ERR_REQUEST_ERROR: return "request error";
    case RTDE_ERR_RAW_SOCK_ERROR: return "raw socket error";
    default: return "unknown RTDE error " + std::to_string(static_cast<int>(result));
  }
}

}  // namespace vendor_robot_driver_core
