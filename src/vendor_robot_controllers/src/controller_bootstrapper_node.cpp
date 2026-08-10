#include <algorithm>
#include <atomic>
#include <chrono>
#include <map>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ControllerBootstrapper : public rclcpp::Node
{
public:
  using ConfigureController = controller_manager_msgs::srv::ConfigureController;
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using LoadController = controller_manager_msgs::srv::LoadController;
  using SwitchController = controller_manager_msgs::srv::SwitchController;

  ControllerBootstrapper()
  : Node("controller_bootstrapper")
  {
    controllers_ = declare_parameter<std::vector<std::string>>(
      "controllers",
      {"joint_state_broadcaster", "driver_status_broadcaster",
        "speed_scaling_broadcaster", "io_and_status_controller", "arm_controller"});
    always_active_ = declare_parameter<std::vector<std::string>>(
      "always_active_controllers",
      {"joint_state_broadcaster", "driver_status_broadcaster",
        "speed_scaling_broadcaster", "io_and_status_controller"});
    arm_controller_ = declare_parameter<std::string>("arm_controller", "arm_controller");
    start_arm_controller_ = declare_parameter<bool>("start_arm_controller", true);
    poll_period_ms_ = declare_parameter<int>("poll_period_ms", 2000);
    request_timeout_ms_ = declare_parameter<int>("request_timeout_ms", 3000);

    if (controllers_.empty() || poll_period_ms_ <= 0 || request_timeout_ms_ <= 0) {
      throw std::invalid_argument(
        "controllers must not be empty and poll/request timeouts must be positive");
    }

    list_client_ = create_client<ListControllers>("controller_manager/list_controllers");
    load_client_ = create_client<LoadController>("controller_manager/load_controller");
    configure_client_ = create_client<ConfigureController>(
      "controller_manager/configure_controller");
    switch_client_ = create_client<SwitchController>("controller_manager/switch_controller");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_period_ms_),
      std::bind(&ControllerBootstrapper::tick, this));
  }

private:
  static bool is_active(const std::string & state)
  {
    return state == "active" || state == "running";
  }

  static bool is_unconfigured(const std::string & state)
  {
    return state.empty() || state == "unconfigured";
  }

  bool begin_request()
  {
    const auto now = std::chrono::steady_clock::now();
    if (request_in_flight_.load(std::memory_order_acquire) &&
      now - request_sent_stamp_ > std::chrono::milliseconds(request_timeout_ms_))
    {
      ++request_generation_;
      request_in_flight_.store(false, std::memory_order_release);
      manager_was_unavailable_ = true;
      RCLCPP_WARN(
        get_logger(),
        "controller_manager request timed out; invalidating the late response and retrying");
    }
    if (request_in_flight_.exchange(true, std::memory_order_acq_rel)) {
      return false;
    }
    request_sent_stamp_ = now;
    return true;
  }

  void tick()
  {
    if (!list_client_->service_is_ready()) {
      manager_was_unavailable_ = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "controller_manager is unavailable; waiting to rebuild the controller set");
      return;
    }
    if (!begin_request()) {
      return;
    }

    const uint64_t generation = ++request_generation_;
    auto request = std::make_shared<ListControllers::Request>();
    list_client_->async_send_request(
      request,
      [this, generation](rclcpp::Client<ListControllers>::SharedFuture future) {
        if (generation != request_generation_) {
          return;
        }
        request_in_flight_.store(false, std::memory_order_release);
        try {
          process_list(future.get());
        } catch (const std::exception & exception) {
          manager_was_unavailable_ = true;
          RCLCPP_WARN(
            get_logger(), "list_controllers failed: %s", exception.what());
        }
      });
  }

  void process_list(const ListControllers::Response::SharedPtr & response)
  {
    std::map<std::string, std::string> states;
    for (const auto & controller : response->controller) {
      states[controller.name] = controller.state;
    }

    std::size_t present_count = 0;
    for (const auto & name : controllers_) {
      present_count += states.count(name) != 0 ? 1U : 0U;
    }

    if (present_count == 0 ||
      (manager_was_unavailable_ && present_count < controllers_.size()) ||
      (first_observation_ && present_count < controllers_.size()))
    {
      bootstrap_cycle_active_ = true;
      RCLCPP_WARN(
        get_logger(),
        "controller_manager appears to have restarted; rebuilding controllers. "
        "arm_controller startup policy remains %s",
        start_arm_controller_ ? "active" : "inactive");
    }
    manager_was_unavailable_ = false;
    first_observation_ = false;

    for (const auto & name : controllers_) {
      if (states.count(name) == 0) {
        load_controller(name);
        return;
      }
    }

    for (const auto & name : controllers_) {
      if (is_unconfigured(states[name])) {
        configure_controller(name);
        return;
      }
    }

    std::vector<std::string> activate;
    for (const auto & name : always_active_) {
      const auto iterator = states.find(name);
      if (iterator != states.end() && !is_active(iterator->second)) {
        activate.push_back(name);
      }
    }

    const auto arm = states.find(arm_controller_);
    if (bootstrap_cycle_active_ && start_arm_controller_ &&
      arm != states.end() && !is_active(arm->second))
    {
      activate.push_back(arm_controller_);
    }

    if (!activate.empty()) {
      activate_controllers(activate);
      return;
    }

    // 仅在完整重建完成后结束 bootstrap 周期。正常运行期间 arm_controller 被
    // ControllerStopper 停用时，它仍然存在，因此本节点不会把它自动重新激活。
    bootstrap_cycle_active_ = false;
  }

  void load_controller(const std::string & name)
  {
    if (!load_client_->service_is_ready() || !begin_request()) {
      return;
    }
    const uint64_t generation = ++request_generation_;
    auto request = std::make_shared<LoadController::Request>();
    request->name = name;
    load_client_->async_send_request(
      request,
      [this, name, generation](rclcpp::Client<LoadController>::SharedFuture future) {
        if (generation != request_generation_) {
          return;
        }
        request_in_flight_.store(false, std::memory_order_release);
        try {
          if (!future.get()->ok) {
            RCLCPP_ERROR(get_logger(), "Failed to load controller %s", name.c_str());
          }
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            get_logger(), "Loading controller %s failed: %s",
            name.c_str(), exception.what());
        }
      });
  }

  void configure_controller(const std::string & name)
  {
    if (!configure_client_->service_is_ready() || !begin_request()) {
      return;
    }
    const uint64_t generation = ++request_generation_;
    auto request = std::make_shared<ConfigureController::Request>();
    request->name = name;
    configure_client_->async_send_request(
      request,
      [this, name, generation](rclcpp::Client<ConfigureController>::SharedFuture future) {
        if (generation != request_generation_) {
          return;
        }
        request_in_flight_.store(false, std::memory_order_release);
        try {
          if (!future.get()->ok) {
            RCLCPP_ERROR(get_logger(), "Failed to configure controller %s", name.c_str());
          }
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            get_logger(), "Configuring controller %s failed: %s",
            name.c_str(), exception.what());
        }
      });
  }

  void activate_controllers(const std::vector<std::string> & names)
  {
    if (!switch_client_->service_is_ready() || !begin_request()) {
      return;
    }
    const uint64_t generation = ++request_generation_;
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = names;
    request->strictness = request->STRICT;
    request->activate_asap = false;
    switch_client_->async_send_request(
      request,
      [this, names, generation](rclcpp::Client<SwitchController>::SharedFuture future) {
        if (generation != request_generation_) {
          return;
        }
        request_in_flight_.store(false, std::memory_order_release);
        try {
          if (!future.get()->ok) {
            RCLCPP_ERROR(get_logger(), "Failed to activate rebuilt controllers");
            return;
          }
          std::string joined;
          for (const auto & name : names) {
            joined += (joined.empty() ? "" : ", ") + name;
          }
          RCLCPP_INFO(get_logger(), "Activated rebuilt controllers: %s", joined.c_str());
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            get_logger(), "Controller activation failed: %s", exception.what());
        }
      });
  }

  std::vector<std::string> controllers_;
  std::vector<std::string> always_active_;
  std::string arm_controller_;
  bool start_arm_controller_{true};
  int poll_period_ms_{2000};
  int request_timeout_ms_{3000};
  bool manager_was_unavailable_{false};
  bool bootstrap_cycle_active_{false};
  bool first_observation_{true};
  std::atomic<bool> request_in_flight_{false};
  uint64_t request_generation_{0};
  std::chrono::steady_clock::time_point request_sent_stamp_{};

  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  rclcpp::Client<LoadController>::SharedPtr load_client_;
  rclcpp::Client<ConfigureController>::SharedPtr configure_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ControllerBootstrapper>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("controller_bootstrapper"),
      "ControllerBootstrapper failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
