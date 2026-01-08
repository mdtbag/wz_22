#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "robot_interfaces/action/collect.hpp"

using Collect = robot_interfaces::action::Collect;
using GoalHandleCollect = rclcpp_action::ServerGoalHandle<Collect>;

class CollectServer : public rclcpp::Node
{
public:
  CollectServer()
  : Node("robot_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    this->declare_parameter<double>("micro_robot_speed", 3.0);
    micro_speed_ = this->get_parameter("micro_robot_speed").as_double();

    server_ = rclcpp_action::create_server<Collect>(
      this,
      "robot_controller/collect",
      std::bind(&CollectServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CollectServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CollectServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "collect action server started");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Collect::Goal> )
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCollect>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCollect> goal_handle)
  {
    std::thread{std::bind(&CollectServer::execute, this, goal_handle)}.detach();
  }

  bool lookupBaseWorld(double & bx, double & by)
  {
    try {
      auto tf = tf_buffer_.lookupTransform("world", "base_link", tf2::TimePointZero);
      bx = tf.transform.translation.x;
      by = tf.transform.translation.y;
      return true;
    } catch (const std::exception &) {
      bx = 0.0; by = 0.0;
      return false;
    }
  }

  void broadcastMicro(double x, double y){
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = this->get_clock()->now();
    ts.header.frame_id = "world";
    ts.child_frame_id = "micro_base_link";
    ts.transform.translation.x = x;
    ts.transform.translation.y = y;
    ts.transform.translation.z = 0.0;
    ts.transform.rotation.w = 1.0;
    ts.transform.rotation.x = 0.0;
    ts.transform.rotation.y = 0.0;
    ts.transform.rotation.z = 0.0;
    tf_broadcaster_.sendTransform(ts);
  }

  void execute(const std::shared_ptr<GoalHandleCollect> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    double tx = goal->target_x;
    double ty = goal->target_y;

    double mx=0.0, my=0.0;
    (void)lookupBaseWorld(mx, my);

    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(20.0); 

    auto feedback = std::make_shared<Collect::Feedback>();

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<Collect::Result>();
        result->success = false;
        result->message = "collect canceled";
        auto dur = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        result->collection_time = dur;
        goal_handle->canceled(result);
        return;
      }

      double dx = tx - mx;
      double dy = ty - my;
      double dist = std::hypot(dx, dy);

      feedback->distance = dist;
      feedback->micro_robot_x = mx;
      feedback->micro_robot_y = my;
      feedback->status = "moving";
      goal_handle->publish_feedback(feedback);

      broadcastMicro(mx, my);

      if (dist < 0.5) {
        auto result = std::make_shared<Collect::Result>();
        result->success = true;
        result->message = "collect success";
        auto dur = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        result->collection_time = dur;
        goal_handle->succeed(result);
        return;
      }

      // move toward target
      double step = micro_speed_ * (1.0 / 20.0);
      if (dist > 1e-6) {
        mx += step * dx / dist;
        my += step * dy / dist;
      }

      rate.sleep();
    }
  }

private:
  double micro_speed_{3.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp_action::Server<Collect>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollectServer>());
  rclcpp::shutdown();
  return 0;
}