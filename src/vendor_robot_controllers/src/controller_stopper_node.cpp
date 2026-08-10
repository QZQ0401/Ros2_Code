#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vendor_robot_msgs/msg/driver_status.hpp"
#include "vendor_robot_msgs/msg/robot_mode.hpp"
#include "vendor_robot_msgs/msg/safety_mode.hpp"

// ============================================================================
// ControllerStopper — 独占运动 + 手动交接
//
// 原则：ROS 运行期间，机器人只允许由本驱动通过 RTDE cd_rtde_servoj 驱动。
// 出现任何其它运动源时立刻停用运动控制器，且不自动恢复——运动源撤离后
// 控制器保持 inactive，必须由操作员手动激活。
//
// 三路输入：
//   /robot_mode    — 谁在动机器人（模式字）
//   /safety_mode   — 安全事件（急停、防护停止）
//   /driver_status — 是不是我们自己在动（消解 SDK_Moving(102) 的歧义）
//
// 两条执行路径：
//   快速通道 — 事件驱动：topic 到达 → evaluate() → dispatch_stop()
//   巡检通道 — 500ms 定时器：看门狗 / 纠正手动误激活 / 重试失败请求
// ============================================================================

namespace
{
constexpr int32_t kModeBackDrive = 9;          // 反向驱动
constexpr int32_t kModeJog = 100;              // 点动
constexpr int32_t kModeTeach = 101;            // 拖动示教
constexpr int32_t kModeSdkMoving = 102;        // servoj 与 SDK movel/movec 等运动共用该模式
constexpr int32_t kModeProgramStop = 103;      // 上电使能后的正常就绪态，不是外部运动源
constexpr int32_t kModeProgramRunFirst = 104;  // lua 程序：暂停 / 停止中 / 暂停中 / 运行
constexpr int32_t kModeProgramRunLast = 110;
}  // namespace

class ControllerStopper : public rclcpp::Node
{
public:
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using ListControllers = controller_manager_msgs::srv::ListControllers;

  ControllerStopper()
  : Node("controller_stopper")
  {
    // ---- 基础参数 ----
    enabled_ = declare_parameter<bool>("enabled", true);
    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "ControllerStopper disabled; no motion-source checks.");
    }

    controllers_ = declare_parameter<std::vector<std::string>>(
      "controllers", {"arm_controller"});

    // ---- 时序参数 ----
    startup_grace_ms_ = declare_parameter<int>("startup_grace_ms", 3000);
    mode_timeout_ms_ = declare_parameter<int>("mode_timeout_ms", 500);
    if (mode_timeout_ms_ <= 0 || startup_grace_ms_ < mode_timeout_ms_) {
      throw std::invalid_argument(
        "mode_timeout_ms > 0 && startup_grace_ms >= mode_timeout_ms required");
    }

    // ---- 策略参数 ----
    treat_sdk_motion_as_external_ =
      declare_parameter<bool>("treat_sdk_motion_as_external", true);
    auto_restart_ = declare_parameter<bool>("auto_restart", false);
    stop_on_watchdog_ = declare_parameter<bool>("stop_on_watchdog", true);
    enforce_period_ms_ = declare_parameter<int>("enforce_period_ms", 500);
    servo_command_timeout_ms_ =
      declare_parameter<int>("servo_command_timeout_ms", 300);
    switch_timeout_ms_ = declare_parameter<int>("switch_timeout_ms", 500);
    verify_timeout_ms_ = declare_parameter<int>("verify_timeout_ms", 500);
    if (enforce_period_ms_ <= 0 || servo_command_timeout_ms_ <= 0 ||
      switch_timeout_ms_ <= 0 || verify_timeout_ms_ <= 0)
    {
      throw std::invalid_argument(
        "enforce_period_ms, servo_command_timeout_ms and service timeouts must be positive");
    }
    if (controllers_.empty()) {
      throw std::invalid_argument("controllers must not be empty");
    }

    // auto_restart 保留声明只是为了兼容既有 launch（未声明的参数会导致节点启动失败），
    // 本节点在任何取值下都**不会**自动重新激活控制器 —— 手动交接是本策略的前提。
    if (auto_restart_) {
      RCLCPP_ERROR(get_logger(),
        "auto_restart:=true is IGNORED by design. Motion controllers are never "
        "re-activated automatically; the operator must run "
        "'ros2 control set_controller_state <controller> active' after the "
        "external motion source withdraws.");
      auto_restart_ = false;
    }

    // 可配置的外部运动模式列表
    external_motion_modes_ = {
      kModeBackDrive, kModeJog, kModeTeach};
    for (int32_t m = kModeProgramRunFirst; m <= kModeProgramRunLast; ++m) {
      external_motion_modes_.insert(m);
    }
    const auto extra = declare_parameter(
      "external_motion_modes", std::vector<int64_t>{});
    for (const auto v : extra) {
      external_motion_modes_.insert(static_cast<int32_t>(v));
    }
    if (external_motion_modes_.count(kModeProgramStop)) {
      RCLCPP_ERROR(get_logger(),
        "external_motion_modes contains 103 (PROGRAM_STOP), which is the normal "
        "idle-and-enabled state on this controller. Removing it; otherwise "
        "arm_controller could never stay active.");
      external_motion_modes_.erase(kModeProgramStop);
    }

    startup_time_ = std::chrono::steady_clock::now();
    last_robot_stamp_ = startup_time_;
    last_safety_stamp_ = startup_time_;

    // ---- 服务客户端 ----
    switch_client_ = create_client<SwitchController>(
      "controller_manager/switch_controller");
    list_client_ = create_client<ListControllers>(
      "controller_manager/list_controllers");

    // ---- 订阅 ----
    safety_sub_ = create_subscription<vendor_robot_msgs::msg::SafetyMode>(
      "safety_mode", 10,
      [this](const vendor_robot_msgs::msg::SafetyMode::SharedPtr msg) {
        last_safety_stamp_ = std::chrono::steady_clock::now();
        safety_seen_ = true;
        latest_protective_stop_ = msg->protective_stop;
        latest_emergency_stop_ = msg->emergency_stop;
        on_input();
      });

    robot_sub_ = create_subscription<vendor_robot_msgs::msg::RobotMode>(
      "robot_mode", 10,
      [this](const vendor_robot_msgs::msg::RobotMode::SharedPtr msg) {
        last_robot_stamp_ = std::chrono::steady_clock::now();
        robot_seen_ = true;
        latest_mode_ = msg->mode;
        latest_mode_name_ = msg->name;
        latest_servo_enabled_ = msg->servo_enabled;
        latest_program_running_ = msg->program_running;
        on_input();
      });

    driver_sub_ = create_subscription<vendor_robot_msgs::msg::DriverStatus>(
      "driver_status", 10,
      [this](const vendor_robot_msgs::msg::DriverStatus::SharedPtr msg) {
        const auto now = std::chrono::steady_clock::now();
        driver_seen_ = true;
        last_driver_stamp_ = now;
        latest_driver_state_fresh_ = msg->state_fresh;
        latest_driver_connected_ = msg->rtde_connected;
        latest_driver_safety_latched_ = msg->safety_stop_latched;
        if (msg->command_fresh && msg->rtde_connected) {
          last_command_fresh_stamp_ = now;
        }
        on_input();
      });

    // ---- 巡检定时器 ----
    enforce_timer_ = create_wall_timer(
      std::chrono::milliseconds(enforce_period_ms_),
      [this]() {enforce_tick();});

    RCLCPP_INFO(get_logger(),
      "ControllerStopper enabled: treat_sdk_motion_as_external=%s, "
      "auto_restart=%s, enforce_period_ms=%d, servo_command_timeout_ms=%d",
      treat_sdk_motion_as_external_ ? "true" : "false",
      auto_restart_ ? "true" : "false",
      enforce_period_ms_, servo_command_timeout_ms_);
  }

private:
  bool own_servo_active() const
  {
    if (!driver_seen_) {return false;}
    const auto now = std::chrono::steady_clock::now();
    return now - last_command_fresh_stamp_ <=
      std::chrono::milliseconds(servo_command_timeout_ms_);
  }

  std::optional<std::string> evaluate()
  {
    const auto now = std::chrono::steady_clock::now();

    if (safety_seen_) {
      if (latest_emergency_stop_) {
        return "emergency stop";
      }
      if (latest_protective_stop_) {
        return "protective stop";
      }
    }

    // 启动宽限只容忍“状态源尚未到达”，已经明确收到的不安全状态必须立即处理。
    if (driver_seen_ &&
      (!latest_driver_connected_ || !latest_driver_state_fresh_ ||
      latest_driver_safety_latched_))
    {
      return "RTDE driver state is stale or motion safety is latched";
    }

    if (robot_seen_) {
      if (latest_mode_ == kModeSdkMoving) {
        if (own_servo_active()) {
          return std::nullopt;
        }
        if (!treat_sdk_motion_as_external_) {
          return std::nullopt;
        }
        return "SDK motion command (movel/movec) is driving the robot: " +
               latest_mode_name_ + " (mode 102)";
      }

      if (external_motion_modes_.count(latest_mode_)) {
        return "external motion source owns the robot: " + latest_mode_name_ +
               " (mode " + std::to_string(latest_mode_) + ")";
      }

      if (latest_program_running_) {
        return "controller program is running: " + latest_mode_name_;
      }

      if (!latest_servo_enabled_) {
        return "robot mode does not accept servoj: " + latest_mode_name_ +
               " (mode " + std::to_string(latest_mode_) + ")";
      }
    }

    if (now - startup_time_ < std::chrono::milliseconds(startup_grace_ms_)) {
      return std::nullopt;
    }

    if (stop_on_watchdog_) {
      const bool robot_stale = !robot_seen_ ||
        now - last_robot_stamp_ > std::chrono::milliseconds(mode_timeout_ms_);
      const bool safety_stale = !safety_seen_ ||
        now - last_safety_stamp_ > std::chrono::milliseconds(mode_timeout_ms_);
      const bool driver_stale = !driver_seen_ ||
        now - last_driver_stamp_ > std::chrono::milliseconds(mode_timeout_ms_);
      if (robot_stale || safety_stale || driver_stale) {
        return "robot/safety/driver status watchdog expired";
      }
    }

    return std::nullopt;
  }

  void on_input()
  {
    if (!enabled_) {return;}
    const auto reason = evaluate();
    const bool was_unsafe = unsafe_.exchange(reason.has_value());

    // RCLCPP_INFO(get_logger(),
    //   "on_input: mode=%d(%s) unsafe=%d eval=%s",
    //   latest_mode_, latest_mode_name_.c_str(),
    //   was_unsafe,
    //   reason.has_value() ? reason->c_str() : "SAFE");

    if (reason.has_value()) {
      unsafe_reason_ = *reason;
      if (!was_unsafe) {
        RCLCPP_WARN(get_logger(), "Unsafe: %s — dispatching stop", reason->c_str());
      }
      if (!deactivate_issued_) {
        stop_pending_ = true;
        dispatch_stop();
      }
    } else if (was_unsafe) {
      // 危险源消失不能取消尚未确认完成的停控事务。
      announce_manual_handback();
    }
  }

  void enforce_tick()
  {
    if (!enabled_) {return;}
    const auto now = std::chrono::steady_clock::now();

    if (list_in_flight_ &&
      now - list_sent_stamp_ > std::chrono::milliseconds(verify_timeout_ms_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "list_controllers request timed out; re-arming verification");
      ++list_generation_;
      list_in_flight_ = false;
      if (verifying_stop_) {
        verifying_stop_ = false;
        stop_pending_ = true;
      }
    }

    if (request_in_flight_ &&
      now - request_sent_stamp_ > std::chrono::milliseconds(switch_timeout_ms_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "switch_controller request timed out; re-arming stop request");
      ++switch_generation_;
      request_in_flight_ = false;
      stop_pending_ = true;
    }

    // 已锁存的停控事务优先于当前模式是否已经恢复安全。
    if (stop_pending_) {
      dispatch_stop();
      return;
    }

    const auto reason = evaluate();
    const bool now_unsafe = reason.has_value();

    if (now_unsafe) {
      unsafe_reason_ = *reason;
      if (!unsafe_.exchange(true)) {
        RCLCPP_WARN(get_logger(), "Unsafe: %s", unsafe_reason_.c_str());
        stop_pending_ = true;
      }
      if (stop_pending_) {
        dispatch_stop();
        return;
      }

      if (list_in_flight_) {return;}
      if (!list_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "list_controllers unavailable; cannot verify that motion controllers "
          "stay inactive");
        return;
      }
      list_in_flight_ = true;
      list_sent_stamp_ = now;
      const uint64_t generation = ++list_generation_;
      auto request = std::make_shared<ListControllers::Request>();
      list_client_->async_send_request(
        request, [this, generation](rclcpp::Client<ListControllers>::SharedFuture future) {
          if (generation != list_generation_) {
            return;
          }
          list_in_flight_ = false;
          try {
            for (const auto & c : future.get()->controller) {
              const bool managed =
                std::find(controllers_.begin(), controllers_.end(), c.name) !=
                controllers_.end();
              const bool active = c.state == "active" || c.state == "running";
              if (managed && active && unsafe_) {
                RCLCPP_WARN(get_logger(),
                  "Controller %s was manually activated while unsafe (%s); "
                  "deactivating again", c.name.c_str(), unsafe_reason_.c_str());
                deactivate_issued_ = false;
                stop_pending_ = true;
                dispatch_stop();
                return;
              }
            }
          } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "list_controllers failed: %s", e.what());
          }
        });
    } else {
      if (unsafe_.exchange(false)) {
        announce_manual_handback();
      }
    }
  }

  void announce_manual_handback()
  {
    unsafe_reason_.clear();
    std::string names;
    for (std::size_t i = 0; i < controllers_.size(); ++i) {
      names += (i > 0 ? " " : "") + controllers_[i];
    }

    if (stop_pending_ || request_in_flight_ || verifying_stop_) {
      RCLCPP_WARN(get_logger(),
        "The unsafe condition cleared, but controller deactivation is still pending. "
        "The stop transaction remains latched until inactive state is confirmed.");
      return;
    }

    // 已完成停控且危险源撤离后，重新武装下一次危险事件；不自动激活控制器。
    deactivate_issued_ = false;
    RCLCPP_INFO(get_logger(),
      "External motion source withdrew and the robot is ready again. "
      "Motion controllers stay INACTIVE by design; re-activate them manually "
      "when you are ready: ros2 control set_controller_state %s active",
      names.c_str());
  }

  void dispatch_stop()
  {
    if (!stop_pending_ || verifying_stop_) {return;}
    if (request_in_flight_.exchange(true)) {return;}
    if (!switch_client_->service_is_ready()) {
      request_in_flight_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "switch_controller service NOT READY; will retry on enforce_tick");
      return;
    }

    // 停控事务优先于普通巡检。使已有 list_controllers 巡检响应失效，
    // 避免它占用 list_in_flight_ 阻塞后续 inactive 二次确认。
    ++list_generation_;
    list_in_flight_ = false;

    request_sent_stamp_ = std::chrono::steady_clock::now();
    const uint64_t generation = ++switch_generation_;
    auto request = std::make_shared<SwitchController::Request>();
    request->deactivate_controllers = controllers_;
    request->strictness = request->STRICT;
    request->activate_asap = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Stopping motion controllers: %s", unsafe_reason_.c_str());
    const std::string reason = unsafe_reason_;

    switch_client_->async_send_request(
      request,
      [this, reason, generation](rclcpp::Client<SwitchController>::SharedFuture future) {
        if (generation != switch_generation_) {
          return;  // 超时后的迟到响应不得污染新事务
        }
        request_in_flight_ = false;
        try {
          if (!future.get()->ok) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Deactivate request was not accepted; verifying actual controller state"
            );
            verifying_stop_ = true;
            dispatch_stop_verification(reason);
            return;
          }
          verifying_stop_ = true;
          dispatch_stop_verification(reason);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Deactivate request failed: %s; retrying", e.what());
          stop_pending_ = true;
        }
      });
  }

  void dispatch_stop_verification(const std::string & reason)
  {
    if (list_in_flight_.exchange(true)) {return;}
    if (!list_client_->service_is_ready()) {
      list_in_flight_ = false;
      verifying_stop_ = false;
      stop_pending_ = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "list_controllers service NOT READY; cannot confirm deactivation");
      return;
    }

    list_sent_stamp_ = std::chrono::steady_clock::now();
    const uint64_t generation = ++list_generation_;
    auto request = std::make_shared<ListControllers::Request>();
    list_client_->async_send_request(
      request,
      [this, reason, generation](rclcpp::Client<ListControllers>::SharedFuture future) {
        if (generation != list_generation_) {
          return;
        }
        list_in_flight_ = false;
        verifying_stop_ = false;
        try {
          bool any_active = false;
          for (const auto & controller : future.get()->controller) {
            const bool managed =
              std::find(controllers_.begin(), controllers_.end(), controller.name) !=
              controllers_.end();
            const bool active =
              controller.state == "active" || controller.state == "running";
            any_active = any_active || (managed && active);
          }
          if (any_active) {
            RCLCPP_ERROR(get_logger(),
              "Deactivate service returned success but a managed controller is still active; retrying");
            stop_pending_ = true;
            return;
          }

          stop_pending_ = false;
          if (!deactivate_issued_) {
            deactivate_issued_ = true;
            RCLCPP_WARN(get_logger(),
              "Motion controllers confirmed inactive (%s). They will NOT be re-activated automatically.",
              reason.c_str());
          }
          if (!unsafe_.load()) {
            announce_manual_handback();
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "Deactivate verification failed: %s; retrying", e.what());
          stop_pending_ = true;
        }
      });
  }

  // ---- 配置参数 ----
  bool enabled_{true};
  bool treat_sdk_motion_as_external_{true};
  bool auto_restart_{false};
  bool stop_on_watchdog_{true};
  int startup_grace_ms_{3000};
  int mode_timeout_ms_{500};
  int enforce_period_ms_{500};
  int servo_command_timeout_ms_{300};
  int switch_timeout_ms_{500};
  int verify_timeout_ms_{500};
  std::vector<std::string> controllers_;
  std::set<int32_t> external_motion_modes_;

  // ---- 最新输入快照 ----
  int32_t latest_mode_{0};
  std::string latest_mode_name_{"UNKNOWN"};
  bool latest_servo_enabled_{false};
  bool latest_program_running_{false};
  bool latest_protective_stop_{false};
  bool latest_emergency_stop_{false};

  // ---- 状态标志 ----
  std::atomic<bool> unsafe_{false};
  std::atomic<bool> stop_pending_{false};
  std::atomic<bool> request_in_flight_{false};
  std::atomic<bool> list_in_flight_{false};
  bool verifying_stop_{false};
  uint64_t switch_generation_{0};
  uint64_t list_generation_{0};
  std::string unsafe_reason_;
  bool deactivate_issued_{false};

  // ---- 心跳追踪 ----
  bool robot_seen_{false};
  bool safety_seen_{false};
  bool driver_seen_{false};
  std::chrono::steady_clock::time_point startup_time_;
  std::chrono::steady_clock::time_point last_robot_stamp_;
  std::chrono::steady_clock::time_point last_safety_stamp_;
  std::chrono::steady_clock::time_point last_command_fresh_stamp_;
  std::chrono::steady_clock::time_point last_driver_stamp_;
  bool latest_driver_state_fresh_{false};
  bool latest_driver_connected_{false};
  bool latest_driver_safety_latched_{false};
  std::chrono::steady_clock::time_point request_sent_stamp_;
  std::chrono::steady_clock::time_point list_sent_stamp_;

  // ---- 客户端 / 订阅 / 定时器 ----
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  rclcpp::Subscription<vendor_robot_msgs::msg::SafetyMode>::SharedPtr safety_sub_;
  rclcpp::Subscription<vendor_robot_msgs::msg::RobotMode>::SharedPtr robot_sub_;
  rclcpp::Subscription<vendor_robot_msgs::msg::DriverStatus>::SharedPtr driver_sub_;
  rclcpp::TimerBase::SharedPtr enforce_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ControllerStopper>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("controller_stopper"),
      "ControllerStopper failed to start: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
