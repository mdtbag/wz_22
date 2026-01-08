#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <optional>
#include <random>

using std::placeholders::_1;

class TurtleEscape : public rclcpp::Node {
public:
  TurtleEscape() : Node("turtle_escape") {

    self_name_ = this->declare_parameter<std::string>("self_name", "turtle2");
    chaser_name_ = this->declare_parameter<std::string>("chaser_name", "turtle1");

    escape_speed_ = this->declare_parameter<double>("escape_speed", 0.8);
    max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 2.0);
    min_distance_ = this->declare_parameter<double>("min_distance", 0.2); 

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + self_name_ + "/cmd_vel", 10);

    sub_self_ = this->create_subscription<turtlesim::msg::Pose>(
        "/" + self_name_ + "/pose", 10,
        std::bind(&TurtleEscape::onSelfPose, this, _1));
    sub_chaser_ = this->create_subscription<turtlesim::msg::Pose>(
        "/" + chaser_name_ + "/pose", 10,
        std::bind(&TurtleEscape::onChaserPose, this, _1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TurtleEscape::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "TurtleEscape started: %s fleeing from %s",
                self_name_.c_str(), chaser_name_.c_str());
  }

private:
  void onSelfPose(const turtlesim::msg::Pose::SharedPtr msg) {
    self_pose_ = *msg;
  }

  void onChaserPose(const turtlesim::msg::Pose::SharedPtr msg) {
    chaser_pose_ = *msg;
  }

  bool out_of_range(double x,double y){
    if(std::abs(x) >=11 or std::abs(y)>=11){
      return 1;
    }
    return false;
  }
  void controlLoop() {
    if (!self_pose_.has_value() || !chaser_pose_.has_value()) {
      return;
    }

    const auto& self = self_pose_.value();
    const auto& chaser = chaser_pose_.value();

    double dx = self.x - chaser.x;
    double dy = self.y - chaser.y;
    double dist = std::hypot(dx, dy);

    
    geometry_msgs::msg::Twist cmd;

    // 如果追击者太远，就停下来（或随机游走）
    if (dist > min_distance_) {

      double escape_angle = std::atan2(dy, dx);
      double angle_error = normalizeAngle(escape_angle - self.theta);

      cmd.linear.x = escape_speed_;
      cmd.angular.z = 2.0 * angle_error; // 比例控制转向

      // 限制角速度
      cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_speed_, max_angular_speed_);
      if(out_of_range(self.x,self.y)){
        RCLCPP_INFO(this->get_logger(),"out_of_range");
        cmd.linear.x = -escape_speed_;
      }
    } else {
      RCLCPP_INFO(this->get_logger(),"I HAVE BEEN CAUGHT");
      cmd.linear.x = 0;
      cmd.angular.z = -2.0;
    }

    pub_cmd_->publish(cmd);
  }

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  // Parameters
  std::string self_name_;
  std::string chaser_name_;
  double escape_speed_{0.8};
  double max_angular_speed_{2.0};
  double min_distance_{0.5};


  std::optional<turtlesim::msg::Pose> self_pose_;
  std::optional<turtlesim::msg::Pose> chaser_pose_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_self_, sub_chaser_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleEscape>());
  rclcpp::shutdown();
  return 0;
}