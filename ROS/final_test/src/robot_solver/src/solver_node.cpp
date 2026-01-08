#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "robot_interfaces/msg/target.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robot_interfaces/msg/control.hpp"
#include "robot_interfaces/srv/set_mode.hpp"

class SolverNode : public rclcpp::Node
{
public:
  SolverNode()
  : Node("robot_solver"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // doc required params
    this->declare_parameter<double>("bullet_speed", 20.0);
    this->declare_parameter<std::string>("hit_mode", "positive");
    this->declare_parameter<double>("fire_angle_threshold_deg", 2.0);
    this->declare_parameter<double>("fire_cooldown_sec", 0.2);

    bullet_speed_ = this->get_parameter("bullet_speed").as_double();
    hit_mode_ = toLower(this->get_parameter("hit_mode").as_string());
    fire_angle_th_deg_ = this->get_parameter("fire_angle_threshold_deg").as_double();
    fire_cooldown_sec_ = this->get_parameter("fire_cooldown_sec").as_double();

    pub_ = this->create_publisher<robot_interfaces::msg::Control>("robot_solver/control", 10);
    sub_ = this->create_subscription<robot_interfaces::msg::Target>(
      "robot_tracker/target", 10,
      std::bind(&SolverNode::onTarget, this, std::placeholders::_1)
    );

    srv_ = this->create_service<robot_interfaces::srv::SetMode>(
      "robot_solver/set_mode",
      std::bind(&SolverNode::onSetMode, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "robot_solver started, mode=%s", hit_mode_.c_str());
  }

private:
  static std::string toLower(std::string s){
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return s;
  }

  bool lookupYawRad(const std::string & target_frame, const std::string & source_frame, double & yaw_rad)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      yaw_rad = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  void onSetMode(
    const std::shared_ptr<robot_interfaces::srv::SetMode::Request> req,
    std::shared_ptr<robot_interfaces::srv::SetMode::Response> res){
    auto m = toLower(req->hit_mode);
    if (m != "positive" && m != "passive") {
      res->success = false;
      res->message = "mode must be 'positive' or 'passive'";
      return;
    }
    hit_mode_ = m;
    res->success = true;
    res->message = "mode set to " + m;
    RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
  }

  void onTarget(const robot_interfaces::msg::Target & t)
  {
    double x = t.x, y = t.y, vx = t.vx, vy = t.vy;
    double dist = std::hypot(x, y);
    double speed = std::hypot(vx, vy);

    // Predict with t = dist / bullet_speed (doc says bullet speed constant, no gravity) :contentReference[oaicite:11]{index=11}
    double tfly = dist / std::max(1e-6, bullet_speed_);
    double xp = x + vx * tfly;
    double yp = y + vy * tfly;

    double offset_rad = std::atan2(yp, xp);
    double offset_deg = offset_rad * 180.0 / M_PI;

    // current gimbal yaw in base frame: base_link -> gimbal_link
    double gimbal_yaw_base = 0.0;
    if (!lookupYawRad("base_link", "gimbal_link", gimbal_yaw_base)) return;

    // desired yaw in base frame
    double aim_yaw_base = gimbal_yaw_base;
    if (hit_mode_ == "positive") {
      aim_yaw_base = gimbal_yaw_base + offset_rad;
    } else {
      aim_yaw_base = gimbal_yaw_base; 
    }

    // try get world->base_link yaw (optional in doc), if not exist assume 0 :contentReference[oaicite:12]{index=12}
    double base_yaw_world = 0.0;
    (void)lookupYawRad("world", "base_link", base_yaw_world);

    double aim_yaw_world = base_yaw_world + aim_yaw_base;

    // fire rule: when aligned (small angle). also stop fire if target stopped (doc says stop firing when destroyed) :contentReference[oaicite:13]{index=13}
    bool fire = false;
    if (speed > 1e-3) {
      bool angle_ok = std::abs(offset_deg) < fire_angle_th_deg_;
      auto now = this->get_clock()->now();
      double dt = (now - last_fire_time_).seconds();
      if (angle_ok && dt > fire_cooldown_sec_) {
        fire = true;
        last_fire_time_ = now;
      }
    }

    robot_interfaces::msg::Control out;
    out.yaw_angle = aim_yaw_world * 180.0 / M_PI; // degrees
    out.distance = dist;
    out.fire = fire;
    out.hit_mode = hit_mode_;
    pub_->publish(out);
  }

private:
  rclcpp::Publisher<robot_interfaces::msg::Control>::SharedPtr pub_;
  rclcpp::Subscription<robot_interfaces::msg::Target>::SharedPtr sub_;
  rclcpp::Service<robot_interfaces::srv::SetMode>::SharedPtr srv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double bullet_speed_{20.0};
  std::string hit_mode_{"positive"};

  double fire_angle_th_deg_{2.0};
  double fire_cooldown_sec_{0.2};
  rclcpp::Time last_fire_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SolverNode>());
  rclcpp::shutdown();
  return 0;
}
