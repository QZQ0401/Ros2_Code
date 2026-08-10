#include <atomic>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vendor_robot_msgs/action/move_l.hpp"

class MoveLActionServer : public rclcpp::Node
{
public:
  using MoveL = vendor_robot_msgs::action::MoveL;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveL>;

  MoveLActionServer() : Node("move_l_action_server")
  {
    group_name_ = declare_parameter<std::string>("group_name", "arm");
    min_fraction_ = declare_parameter<double>("min_cartesian_fraction", 0.995);
    eef_step_ = declare_parameter<double>("eef_step", 0.005);
    workspace_radius_ = declare_parameter<double>("workspace_radius", 1.5);
  }

  ~MoveLActionServer() override
  {
    cancel_requested_.store(true, std::memory_order_release);
    if (move_group_) {
      // MoveIt 的 stop() 用于中断正在执行的轨迹。先请求停止，再等待受管线程退出，
      // 避免 detached thread 在节点析构后继续访问 this / move_group_。
      move_group_->stop();
    }
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

  void initialize()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), group_name_);
    server_ = rclcpp_action::create_server<MoveL>(
      shared_from_this(), "move_l",
      std::bind(&MoveLActionServer::goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveLActionServer::cancel, this, std::placeholders::_1),
      std::bind(&MoveLActionServer::accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveL::Goal> goal)
  {
    const auto & p = goal->target.pose.position;
    const auto & q = goal->target.pose.orientation;
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    const double radius = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    const bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
      std::isfinite(goal->velocity) && std::isfinite(goal->acceleration);
    if (!finite || norm < 1e-6 || radius > workspace_radius_ ||
      goal->velocity <= 0.0 || goal->velocity > 1.0 ||
      goal->acceleration <= 0.0 || goal->acceleration > 1.0)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel(const std::shared_ptr<GoalHandle>)
  {
    cancel_requested_.store(true);
    if (move_group_) {
      // execute() 可能阻塞在 MoveIt 执行接口中，stop() 是其公开取消入口。
      move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void accepted(const std::shared_ptr<GoalHandle> handle)
  {
    // busy_ 保证同一时刻只有一个目标。上一个线程正常结束后先 join，
    // 再启动新的受管线程，禁止 detached 生命周期逃逸。
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    execution_thread_ = std::thread([this, handle] {execute(handle);});
  }

  void execute(const std::shared_ptr<GoalHandle> handle)
  {
    struct BusyReset
    {
      std::atomic<bool> & flag;
      ~BusyReset() {flag.store(false);}
    } busy_reset{busy_};

    cancel_requested_.store(false);
    auto result = std::make_shared<MoveL::Result>();
    auto feedback = std::make_shared<MoveL::Feedback>();

    try {
      const auto goal = handle->get_goal();
      auto normalized_target = goal->target.pose;
      const double quaternion_norm = std::sqrt(
        normalized_target.orientation.x * normalized_target.orientation.x +
        normalized_target.orientation.y * normalized_target.orientation.y +
        normalized_target.orientation.z * normalized_target.orientation.z +
        normalized_target.orientation.w * normalized_target.orientation.w);
      normalized_target.orientation.x /= quaternion_norm;
      normalized_target.orientation.y /= quaternion_norm;
      normalized_target.orientation.z /= quaternion_norm;
      normalized_target.orientation.w /= quaternion_norm;

      std::vector<geometry_msgs::msg::Pose> waypoints{normalized_target};
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = 0.0;
      {
        // 配置、规划和状态读取串行化；真正的 execute() 不持有该锁，
        // cancel 回调可以通过 MoveIt 的公开 stop() 接口中断轨迹执行。
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        move_group_->setPoseReferenceFrame(goal->target.header.frame_id.empty() ?
          move_group_->getPlanningFrame() : goal->target.header.frame_id);
        move_group_->setMaxVelocityScalingFactor(goal->velocity);
        move_group_->setMaxAccelerationScalingFactor(goal->acceleration);
        fraction = move_group_->computeCartesianPath(waypoints, eef_step_, 0.0, trajectory);
        feedback->actual = move_group_->getCurrentPose();
      }

      result->trajectory_fraction = fraction;
      feedback->progress = 0.5F;
      handle->publish_feedback(feedback);

      if (cancel_requested_.load() || handle->is_canceling()) {
        result->success = false;
        result->error_code = -1;
        result->message = "canceled during planning";
        handle->canceled(result);
      } else if (fraction < min_fraction_) {
        result->success = false;
        result->error_code = -2;
        result->message = "Cartesian fraction below acceptance threshold";
        handle->abort(result);
      } else {
        const auto code = move_group_->execute(trajectory);
        result->error_code = code.val;
        result->success = code == moveit::core::MoveItErrorCode::SUCCESS;
        result->message = result->success ? "completed" : "MoveIt execution failed";
        if (cancel_requested_.load() || handle->is_canceling()) {
          handle->canceled(result);
        } else if (result->success) {
          feedback->progress = 1.0F;
          {
            std::lock_guard<std::mutex> lock(move_group_mutex_);
            feedback->actual = move_group_->getCurrentPose();
          }
          handle->publish_feedback(feedback);
          handle->succeed(result);
        } else {
          handle->abort(result);
        }
      }

      {
        std::lock_guard<std::mutex> lock(move_group_mutex_);
        move_group_->clearPoseTargets();
      }
    } catch (const std::exception & exception) {
      result->success = false;
      result->error_code = -3;
      result->message = std::string("MoveIt exception: ") + exception.what();
      handle->abort(result);
    }
  }

  std::string group_name_;
  double min_fraction_{0.995};
  double eef_step_{0.005};
  double workspace_radius_{1.5};
  std::atomic<bool> busy_{false};
  std::atomic<bool> cancel_requested_{false};
  std::thread execution_thread_;
  std::mutex move_group_mutex_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Server<MoveL>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveLActionServer>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
