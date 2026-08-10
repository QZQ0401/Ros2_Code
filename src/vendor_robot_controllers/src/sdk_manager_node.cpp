#include <algorithm>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vendor_robot_msgs/msg/robot_mode.hpp"
#include "vendor_robot_msgs/msg/safety_mode.hpp"
#include "vendor_robot_msgs/srv/set_io.hpp"
#include "vendor_robot_msgs/srv/set_payload.hpp"

extern "C"
{
#include "basestruct.h"
#include "robotapi.h"
}

using namespace std::chrono_literals;

namespace
{
const char * sdk_error_message(CRresult result)
{
  switch (result) {
    case success: return "success";
    case error: return "general error";
    case thread_running: return "connection already exists";
    case operate_timeout: return "operation timed out";
    case result_invalid: return "invalid or empty RPC result";
    case out_of_range: return "value out of range";
    case mutex_invalid: return "internal mutex error";
    case para_error: return "invalid parameter";
    case no_result: return "result not found";
    case no_handle: return "robot handle not created";
    case handle_repeat: return "duplicate robot handle";
    case robotmode_error: return "operation rejected in current robot mode";
    default: return "SDK error";
  }
}
}  // namespace

class SdkManager : public rclcpp::Node
{
public:
  SdkManager() : Node("sdk_manager")
  {
    robot_ip_ = declare_parameter<std::string>("robot_ip", "192.168.6.6");
    port_ = declare_parameter<int>("sdk_port", 2323);
    password_ = declare_parameter<std::string>("sdk_password", "");
    poll_period_ms_ = declare_parameter<int>("poll_period_ms", 100);
    status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    robot_mode_publisher_ =
      create_publisher<vendor_robot_msgs::msg::RobotMode>("robot_mode", 10);
    safety_mode_publisher_ =
      create_publisher<vendor_robot_msgs::msg::SafetyMode>("safety_mode", 10);
    clear_error_service_ = create_service<std_srvs::srv::Trigger>(
      "clear_error", std::bind(
        &SdkManager::clear_error, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);
    power_on_service_ = create_service<std_srvs::srv::Trigger>(
      "power_on",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        execute_simple("power on", cr_poweron, response);
      }, rmw_qos_profile_services_default, service_callback_group_);
    enable_service_ = create_service<std_srvs::srv::Trigger>(
      "enable_robot",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        execute_simple("enable", cr_enable, response);
      }, rmw_qos_profile_services_default, service_callback_group_);
    disable_service_ = create_service<std_srvs::srv::Trigger>(
      "disable_robot",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        execute_simple("disable", cr_disable, response);
      }, rmw_qos_profile_services_default, service_callback_group_);
    stop_program_service_ = create_service<std_srvs::srv::Trigger>(
      "stop_program",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
      {
        execute_simple("stop program", cr_stop, response);
      }, rmw_qos_profile_services_default, service_callback_group_);
    tool_power_service_ = create_service<std_srvs::srv::SetBool>(
      "set_tool_power",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
      {
        std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
        if (!lock.try_lock_for(100ms)) {
          response->success = false;
          response->message = "SDK management channel is busy";
          return;
        }
        if (!ensure_connected()) {
          response->success = false;
          response->message = "SDK disconnected";
          return;
        }
        const CRresult result = cr_set_ToolOutputVoltage(
          handle_, request->data ? Power_on : Power_off);
        response->success = result == success;
        response->message = response->success ? "tool power updated" :
          "SDK error " + std::to_string(static_cast<int>(result));
      }, rmw_qos_profile_services_default, service_callback_group_);
    // SDK IO 服务 —— 通过 SDK 管理通道（端口 2323）设置数字输出。
    // RTDE 通道的 cd_rtde_*digitalout_set 与 250 Hz 状态流共用一个线程和
    // socket，同步阻塞调用会饿死 receive_state()，触发状态看门狗和运动停用。
    // SDK 通道有独立连接，不会影响 RTDE 实时状态流。
    set_sdk_io_service_ = create_service<vendor_robot_msgs::srv::SetIO>(
      "set_sdk_io",
      [this](
        const std::shared_ptr<vendor_robot_msgs::srv::SetIO::Request> request,
        std::shared_ptr<vendor_robot_msgs::srv::SetIO::Response> response)
      {
        std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
        if (!lock.try_lock_for(100ms)) {
          response->success = false;
          response->error_code = operate_timeout;
          response->message = "SDK management channel is busy";
          return;
        }
        if (!ensure_connected()) {
          response->success = false;
          response->error_code = -3;
          response->message = "SDK disconnected";
          return;
        }
        CRresult result = para_error;
        if (request->domain == request->STANDARD_DIGITAL_OUT && request->channel < 8) {
          result = cr_set_stdDigitalOut(
            handle_, static_cast<int>(request->channel), request->value ? TRUE : FALSE);
        } else if (request->domain == request->CONFIGURABLE_DIGITAL_OUT && request->channel < 8) {
          result = cr_set_configDigitalOut(
            handle_, static_cast<int>(request->channel), request->value ? TRUE : FALSE);
        } else if (request->domain == request->TOOL_DIGITAL_OUT && request->channel < 10) {
          result = cr_set_toolDigitalOut(
            handle_, static_cast<int>(request->channel), request->value ? TRUE : FALSE);
        }
        response->success = result == success;
        response->error_code = static_cast<int32_t>(result);
        response->message = response->success ?
          "IO set via SDK" :
          "SDK error " + std::to_string(static_cast<int>(result)) +
          " (" + std::string(sdk_error_message(result)) + ")";
      }, rmw_qos_profile_services_default, service_callback_group_);
    set_payload_service_ = create_service<vendor_robot_msgs::srv::SetPayload>(
      "set_payload", std::bind(
        &SdkManager::set_payload, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(20, poll_period_ms_)),
      std::bind(&SdkManager::poll, this), status_callback_group_);
  }

  ~SdkManager() override
  {
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    if (!lock.try_lock_for(100ms)) {
      // 某个闭源 SDK 调用可能永久阻塞。析构阶段不再等待同一互斥锁，
      // 让进程退出时由 OS 回收连接资源。
      return;
    }
    if (handle_ >= 0) {
      (void)cr_destroy_robot(handle_);
      handle_ = -1;
    }
  }

private:
  bool ensure_connected()
  {
    if (handle_ >= 0) {
      return true;
    }
    RobotHandle next = -1;
    const CRresult result =
      cr_create_robot(&next, robot_ip_.c_str(), port_, password_.c_str());
    if (result != success || next < 0) {
      last_error_ = static_cast<int>(result);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "SDK connection to %s:%d failed: %d (%s)",
        robot_ip_.c_str(), port_, last_error_, sdk_error_message(result));
      return false;
    }
    handle_ = next;
    last_error_ = 0;
    {
      char version[64] = {};
      const CRresult ver_result = cr_get_sdk_version(version);
      RCLCPP_INFO(
        get_logger(), "SDK management channel connected (SDK %s)",
        ver_result == success ? version : "<unknown>");
    }
    return true;
  }

  void mark_disconnected(CRresult result)
  {
    last_error_ = static_cast<int>(result);
    if (handle_ >= 0) {
      (void)cr_destroy_robot(handle_);
      handle_ = -1;
    }
  }

  static std::string robot_mode_name(RobotModes mode)
  {
    switch (mode) {
      case Closed: return "CLOSED";
      case Disconnect: return "DISCONNECTED";
      case ControlerIdle: return "CONTROLLER_IDLE";
      case JointPowerOff: return "POWER_OFF";
      case JointIdle: return "JOINT_IDLE";
      case Enable: return "ENABLED";
      case BackDrive: return "BACKDRIVE";
      case Jog: return "JOG";
      case Teach: return "TEACH";
      case SDK_Moving: return "SDK_MOVING";
      case ProgramStop: return "PROGRAM_STOP";
      case ProgramPause: return "PROGRAM_PAUSE";
      case ProgramRun_MotionStop: return "PROGRAM_RUNNING_MOTION_STOP";
      case ProgramRun_MotionReducing: return "PROGRAM_REDUCING";
      case ProgramRun_MotionMoving: return "PROGRAM_MOVING";
      case ProgramRun_MotionCanBlend: return "PROGRAM_BLEND";
      case Imdstop: return "EMERGENCY_STOP";
      case ProtectiveStop: return "PROTECTIVE_STOP";
      default: return "MODE_" + std::to_string(static_cast<int>(mode));
    }
  }

  void poll()
  {
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    if (!lock.try_lock_for(10ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "SDK status poll skipped because another management call is still running");
      return;
    }
    if (!ensure_connected()) {
      return;
    }
    RobotModes mode = Disconnect;
    const CRresult result = cr_get_robotMode(handle_, &mode);
    if (result != success) {
      mark_disconnected(result);
      return;
    }
    const auto stamp = now();
    vendor_robot_msgs::msg::RobotMode robot;
    robot.header.stamp = stamp;
    robot.mode = static_cast<int32_t>(mode);
    robot.name = robot_mode_name(mode);
    // ProgramStop(103) 是本机型上电使能后的正常就绪态，等价于"已使能、空闲、
    // 可接受 servoj"，并不代表控制柜里有程序占用运动上下文。
    // servo_enabled 与 motion_allowed 取同一判据，避免两份手写列表互相不一致
    // （历史上 motion_allowed 含 109/110 而 servo_enabled 不含，语义冲突）。
    robot.servo_enabled =
      mode == Enable || mode == ProgramStop || mode == SDK_Moving;
    robot.motion_allowed = robot.servo_enabled;
    // program_running 只表示控制柜程序正在执行，供 ControllerStopper 兜底判断
    robot.program_running =
      mode >= ProgramRun_MotionStop && mode <= ProgramRun_MotionCanBlend;

    // 状态切换日志：只在模式变化时打印一次。
    if (mode != previous_mode_) {
      RCLCPP_INFO(get_logger(),
        "Robot mode: %s (%d)",
        robot.name.c_str(), static_cast<int>(mode));
      previous_mode_ = mode;
    }
    robot_mode_publisher_->publish(robot);

    vendor_robot_msgs::msg::SafetyMode safety;
    safety.header.stamp = stamp;
    safety.protective_stop = mode == ProtectiveStop;
    safety.emergency_stop = mode == Imdstop;
    safety.reduced_mode = false;  // SDK header exposes no read API for SafetyModes.
    safety.motion_allowed = robot.motion_allowed &&
      !safety.protective_stop && !safety.emergency_stop;
    safety.mode = safety.emergency_stop ? SAFETY_MODE_SYSTEM_EMERGENCY_STOP :
      (safety.protective_stop ? SAFETY_MODE_PROTECTIVE_STOP : SAFETY_MODE_NORMAL);
    safety.name = safety.emergency_stop ? "EMERGENCY_STOP" :
      (safety.protective_stop ? "PROTECTIVE_STOP" : "NORMAL_INFERRED");
    safety_mode_publisher_->publish(safety);
  }

  void clear_error(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    if (!lock.try_lock_for(100ms)) {
      response->success = false;
      response->message = "SDK management channel is busy";
      return;
    }
    if (!ensure_connected()) {
      response->success = false;
      response->message = "SDK disconnected";
      return;
    }
    const CRresult result = cr_FaultReset(handle_);
    response->success = result == success;
    response->message = response->success ? "fault reset accepted" :
      "SDK error " + std::to_string(static_cast<int>(result));
  }

  using SimpleCall = CRresult (*)(RobotHandle);
  void execute_simple(
    const std::string & operation, SimpleCall call,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    if (!lock.try_lock_for(100ms)) {
      response->success = false;
      response->message = "SDK management channel is busy";
      return;
    }
    if (!ensure_connected()) {
      response->success = false;
      response->message = "SDK disconnected";
      return;
    }
    const CRresult result = call(handle_);
    response->success = result == success;
    response->message = response->success ? operation + " accepted" :
      "SDK error " + std::to_string(static_cast<int>(result));
  }

  void set_payload(
    const std::shared_ptr<vendor_robot_msgs::srv::SetPayload::Request> request,
    std::shared_ptr<vendor_robot_msgs::srv::SetPayload::Response> response)
  {
    if (request->name.empty() || request->name.size() >= 20 ||
      !std::isfinite(request->mass) || request->mass < 0.0 ||
      !std::isfinite(request->center_of_gravity.x) ||
      !std::isfinite(request->center_of_gravity.y) ||
      !std::isfinite(request->center_of_gravity.z))
    {
      response->success = false;
      response->error_code = para_error;
      response->message = "invalid name, mass or center of gravity";
      return;
    }
    std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
    if (!lock.try_lock_for(100ms)) {
      response->success = false;
      response->error_code = operate_timeout;
      response->message = "SDK management channel is busy";
      return;
    }
    if (!ensure_connected()) {
      response->success = false;
      response->error_code = no_handle;
      response->message = "SDK disconnected";
      return;
    }
    PayLoad payload{};
    std::strncpy(payload.payloadName, request->name.c_str(), sizeof(payload.payloadName) - 1);
    payload.toolPayload = request->mass;
    payload.centerOfGravity[0] = request->center_of_gravity.x * 1000.0;
    payload.centerOfGravity[1] = request->center_of_gravity.y * 1000.0;
    payload.centerOfGravity[2] = request->center_of_gravity.z * 1000.0;

    int count = 0;
    CRresult result = cr_cfg_payload_count(handle_, &count);
    int selected = -1;
    if (result == success) {
      for (int index = 0; index < count; ++index) {
        PayLoad existing{};
        if (cr_cfg_payload_get(handle_, index, &existing) == success &&
          request->name == existing.payloadName)
        {
          result = cr_cfg_payload_set(handle_, index, payload);
          selected = index;
          break;
        }
      }
    }
    if (result == success && selected < 0) {
      result = cr_cfg_payload_add(handle_, payload);
      if (result == success && cr_cfg_payload_count(handle_, &count) == success) {
        for (int index = 0; index < count; ++index) {
          PayLoad existing{};
          if (cr_cfg_payload_get(handle_, index, &existing) == success &&
            request->name == existing.payloadName)
          {
            selected = index;
            break;
          }
        }
      }
    }
    if (result == success && selected >= 0) {
      result = cr_cfg_payload_active_set(handle_, selected);
    } else if (result == success) {
      result = no_result;
    }
    response->success = result == success;
    response->error_code = static_cast<int32_t>(result);
    response->message = response->success ? "payload activated" :
      "SDK error " + std::to_string(static_cast<int>(result));
  }

  std::string robot_ip_;
  std::string password_;
  int port_{2323};
  int poll_period_ms_{100};
  RobotModes previous_mode_{Disconnect};
  RobotHandle handle_{-1};
  int last_error_{0};
  std::timed_mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<vendor_robot_msgs::msg::RobotMode>::SharedPtr robot_mode_publisher_;
  rclcpp::Publisher<vendor_robot_msgs::msg::SafetyMode>::SharedPtr safety_mode_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_error_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_on_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_program_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr tool_power_service_;
  rclcpp::Service<vendor_robot_msgs::srv::SetPayload>::SharedPtr set_payload_service_;
  rclcpp::Service<vendor_robot_msgs::srv::SetIO>::SharedPtr set_sdk_io_service_;
};

int main(int argc, char ** argv)
{
  // The closed-source SDK may write to a socket after the peer has closed it.
  // Convert that condition into the SDK return code instead of killing this node.
  std::signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SdkManager>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
