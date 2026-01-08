#include <chrono>
#include <cmath>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_control/action/move_action.hpp"

using namespace std::chrono_literals;

class MoveActionServer : public rclcpp::Node
{
public:
  using MoveAction = robot_control::action::MoveAction;
  using GoalHandleMoveAction = rclcpp_action::ServerGoalHandle<MoveAction>;

  MoveActionServer() : Node("move_action_server"), obstacle_(false)
  {
    speed_ = this->declare_parameter<double>("max_linear_speed", 0.2);

    action_server_ = rclcpp_action::create_server<MoveAction>(
      this,
      "move_to_goal",
      std::bind(&MoveActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveActionServer::handle_accepted, this, std::placeholders::_1));

    sub_obstacle_ = this->create_subscription<std_msgs::msg::String>(
      "obstacle_warning", 10,
      std::bind(&MoveActionServer::obstacle_callback, this, std::placeholders::_1));

    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&MoveActionServer::on_param_set, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveActionServer started, speed=%.2f", speed_);
  }

private:
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const MoveAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal (%.2f, %.2f)", goal->x, goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMoveAction>)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveAction> goal_handle)
  {
    std::thread(
      std::bind(&MoveActionServer::execute, this, std::placeholders::_1),
      goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveAction> goal_handle)
  {
    rclcpp::Rate rate(2.0);  // 0.5s

    auto goal = goal_handle->get_goal();
    double gx = goal->x;
    double gy = goal->y;

    double x = current_x_, y = current_y_;
    auto feedback = std::make_shared<MoveAction::Feedback>();
    auto result = std::make_shared<MoveAction::Result>();

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->final_x = static_cast<float>(x);
        result->final_y = static_cast<float>(y);
        goal_handle->canceled(result);
        return;
      }

      double dx = gx - x;
      double dy = gy - y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist < 0.01) {
        result->success = true;
        result->final_x = static_cast<float>(gx);
        result->final_y = static_cast<float>(gy);
        current_x_ = x;
	current_y_ = y;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal reached");
        return;
      }

      if (obstacle_) {
        result->success = false;
        result->final_x = static_cast<float>(x);
        result->final_y = static_cast<float>(y);
        goal_handle->abort(result);
        RCLCPP_WARN(this->get_logger(), "Aborted because of obstacle");
        return;
      }

      double dt = 0.5;
      double step = speed_ * dt;
      if (step > dist) step = dist;

      double ux = dx / dist;
      double uy = dy / dist;

      x += ux * step;
      y += uy * step;

      feedback->current_x = static_cast<float>(x);
      feedback->current_y = static_cast<float>(y);
      feedback->remaining_distance = static_cast<float>(dist);
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }
  }

  void obstacle_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.find("Obstacle") != std::string::npos) {
      obstacle_ = true;
    } else {
      obstacle_ = false;
    }
  }

  rcl_interfaces::msg::SetParametersResult
  on_param_set(const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      if (p.get_name() == "max_linear_speed") {
        double v = p.as_double();
        if (v <= 0.0) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = false;
          res.reason = "max_linear_speed must be > 0";
          return res;
        }
        speed_ = v;
        RCLCPP_INFO(this->get_logger(), "Updated max_linear_speed=%.2f", speed_);
      }
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  double speed_;
  std::atomic<bool> obstacle_;

  rclcpp_action::Server<MoveAction>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_obstacle_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveActionServer>());
  rclcpp::shutdown();
  return 0;
}
