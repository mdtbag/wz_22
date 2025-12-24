#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_srvs/srv/empty.hpp"
#include "robot_control/srv/robot_state_srv.hpp"

using namespace std::chrono_literals;

class RobotService : public rclcpp::Node
{
public:
  RobotService() : Node("robot_service"), state_("idle"), battery_(100.0f)
  {
    speed_ = this->declare_parameter<double>("max_linear_speed", 0.2);

    srv_get_ = this->create_service<robot_control::srv::RobotStateSrv>(
      "get_robot_state",
      std::bind(&RobotService::handle_get, this, std::placeholders::_1, std::placeholders::_2));

    srv_reset_ = this->create_service<std_srvs::srv::Empty>(
      "reset_robot",
      std::bind(&RobotService::handle_reset, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      1s,
      std::bind(&RobotService::timer_callback, this));

    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&RobotService::on_param_set, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RobotService started, speed=%.2f", speed_);
  }

private:
  void handle_get(
    const std::shared_ptr<robot_control::srv::RobotStateSrv::Request> /*req*/,
    std::shared_ptr<robot_control::srv::RobotStateSrv::Response> res)
  {
    res->state = state_;
    res->battery = battery_;
  }

  void handle_reset(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/)
  {
    state_ = "idle";
    battery_ = 100.0f;
    RCLCPP_INFO(this->get_logger(), "Robot reset");
  }

  void timer_callback()
  {
    if (battery_ > 0.0f) {
      battery_ -= 0.1f;
      if (battery_ < 0.0f) battery_ = 0.0f;
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
        RCLCPP_INFO(this->get_logger(), "Updated max_linear_speed = %.2f", speed_);
      }
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  std::string state_;
  float battery_;
  double speed_;

  rclcpp::Service<robot_control::srv::RobotStateSrv>::SharedPtr srv_get_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotService>());
  rclcpp::shutdown();
  return 0;
}
