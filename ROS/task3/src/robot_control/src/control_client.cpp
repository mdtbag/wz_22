#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "robot_control/srv/robot_state_srv.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cout << "Usage: control_client [get_state | reset]\n";
    return 0;
  }

  auto node = rclcpp::Node::make_shared("control_client");
  std::string cmd = argv[1];

  if (cmd == "get_state") {
    auto client = node->create_client<robot_control::srv::RobotStateSrv>("get_robot_state");
    client->wait_for_service();

    auto req = std::make_shared<robot_control::srv::RobotStateSrv::Request>();
    auto future = client->async_send_request(req);

    if (rclcpp::spin_until_future_complete(node, future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto res = future.get();
      RCLCPP_INFO(node->get_logger(), "state=%s battery=%.1f",
                  res->state.c_str(), res->battery);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Service call failed");
    }
  } else if (cmd == "reset") {
    auto client = node->create_client<std_srvs::srv::Empty>("reset_robot");
    client->wait_for_service();

    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = client->async_send_request(req);
    rclcpp::spin_until_future_complete(node, future);
    RCLCPP_INFO(node->get_logger(), "Reset sent");
  } else {
    std::cout << "Unknown command\n";
  }

  rclcpp::shutdown();
  return 0;
}
