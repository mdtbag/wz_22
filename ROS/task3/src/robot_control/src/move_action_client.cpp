#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_control/action/move_action.hpp"

using namespace std::chrono_literals;

class MoveActionClient : public rclcpp::Node{
public:
  using MoveAction = robot_control::action::MoveAction;
  using GoalHandleMoveAction = rclcpp_action::ClientGoalHandle<MoveAction>;

  MoveActionClient(double x, double y)
  : Node("move_action_client"), gx_(x), gy_(y){
    client_ = rclcpp_action::create_client<MoveAction>(this, "move_to_goal");

    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      throw std::runtime_error("no action server");
    }

    send_goal();
  }

private:
  void send_goal()
  {
    MoveAction::Goal goal;
    goal.x = static_cast<float>(gx_);
    goal.y = static_cast<float>(gy_);

    RCLCPP_INFO(this->get_logger(), "Sending goal (%.2f, %.2f)", gx_, gy_);

    rclcpp_action::Client<MoveAction>::SendGoalOptions options;
    options.feedback_callback = std::bind(
      &MoveActionClient::feedback_callback, this,
      std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(
      &MoveActionClient::result_callback, this,
      std::placeholders::_1);

    client_->async_send_goal(goal, options);
  }

  void feedback_callback(
    GoalHandleMoveAction::SharedPtr,
    const std::shared_ptr<const MoveAction::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "feedback: (%.2f, %.2f), remaining=%.2f",
                feedback->current_x, feedback->current_y,
                feedback->remaining_distance);
  }

  void result_callback(const GoalHandleMoveAction::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Result: success=%d final=(%.2f, %.2f)",
                    result.result->success,
                    result.result->final_x,
                    result.result->final_y);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "Goal aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    rclcpp::shutdown();
  }

  rclcpp_action::Client<MoveAction>::SharedPtr client_;
  double gx_, gy_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cout << "Usage: move_action_client x y\n";
    return 0;
  }

  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);

  try {
    auto node = std::make_shared<MoveActionClient>(x, y);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  return 0;
}
