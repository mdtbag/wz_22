#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <cmath>
#include <optional>

using std::placeholders::_1;

class TurtleChase : public rclcpp::Node {
public:
  TurtleChase() : Node("turtle_chase") {
    // 参数
    target_name_ = this->declare_parameter<std::string>("target_name", "turtle2");
    auto_spawn_  = this->declare_parameter<bool>("auto_spawn_target", true);
    spawn_x_     = this->declare_parameter<double>("spawn_x", 8.0);
    spawn_y_     = this->declare_parameter<double>("spawn_y", 8.0);
    spawn_theta_ = this->declare_parameter<double>("spawn_theta", 0.0);

    kv_ = this->declare_parameter<double>("kv", 1.0);
    kw_ = this->declare_parameter<double>("kw", 4.0);
    v_max_ = this->declare_parameter<double>("v_max", 2.0);
    w_max_ = this->declare_parameter<double>("w_max", 4.0);
    stop_dist_ = this->declare_parameter<double>("stop_distance", 0.2);

    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    // 订阅自身和目标的位姿
    sub_self_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&TurtleChase::cbSelf, this, _1));
    sub_target_ = this->create_subscription<turtlesim::msg::Pose>(
      "/" + target_name_ + "/pose", 10, std::bind(&TurtleChase::cbTarget, this, _1));

    if (auto_spawn_) {
      client_spawn_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
      spawnTarget();
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                     std::bind(&TurtleChase::controlLoop, this));
  }

private:
  void cbSelf(const turtlesim::msg::Pose::SharedPtr msg) {
    self_pose_ = *msg;
  }
  void cbTarget(const turtlesim::msg::Pose::SharedPtr msg) {
    target_pose_ = *msg;
  }

  void spawnTarget() {
    if (!client_spawn_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "spawn service not available yet");
      return;
    }
    auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
    req->x = spawn_x_;
    req->y = spawn_y_;
    req->theta = spawn_theta_;
    req->name = target_name_;
    auto fut = client_spawn_->async_send_request(req,
      [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture f){
        try {
          auto resp = f.get();
          RCLCPP_INFO(this->get_logger(), "Spawned target: %s", resp->name.c_str());
        } catch (...) {
          RCLCPP_WARN(this->get_logger(), "Spawn request failed (maybe exists already)");
        }
      });
    (void)fut;
  }

  void controlLoop() {
    if (!self_pose_.has_value() || !target_pose_.has_value()) return;

    double dx = target_pose_->x - self_pose_->x;
    double dy = target_pose_->y - self_pose_->y;
    double dist = std::hypot(dx, dy);
    double target_yaw = std::atan2(dy, dx); //atan2 比 atan 考虑了四个象限转化为度
    double err_yaw = normalizeAngle(target_yaw - self_pose_->theta);

    geometry_msgs::msg::Twist cmd;

    if (dist < stop_dist_) {
      // 停止
      pub_cmd_->publish(cmd);
      return;
    }

    double v = kv_ * dist;
    double w = kw_ * err_yaw;
    // 限幅
    v = std::clamp(v, -v_max_, v_max_);
    w = std::clamp(w, -w_max_, w_max_);

    cmd.linear.x = v;
    cmd.angular.z = w;
    pub_cmd_->publish(cmd);
  }

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  // params
  std::string target_name_;
  bool auto_spawn_{true};
  double spawn_x_{8.0}, spawn_y_{8.0}, spawn_theta_{0.0};
  double kv_{1.0}, kw_{4.0}, v_max_{2.0}, w_max_{4.0}, stop_dist_{0.2};

  // state
  std::optional<turtlesim::msg::Pose> self_pose_;
  std::optional<turtlesim::msg::Pose> target_pose_;

  // ros
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_self_, sub_target_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_spawn_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleChase>());
  rclcpp::shutdown();
  return 0;
}
