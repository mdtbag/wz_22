#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "robot_control/msg/lidar_data.hpp"

using namespace std::chrono_literals;

class LidarPublisher : public rclcpp::Node{
public:
  LidarPublisher() : Node("lidar_publisher"){
    
    frequency_ = this->declare_parameter<double>("lidar_publish_frequency", 10.0);

    // 创建发布器
    pub_ = this->create_publisher<robot_control::msg::LidarData>("lidar_data", 10);

    // 参数动态修改回调
    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&LidarPublisher::on_param_set, this, std::placeholders::_1));

    // 定时器
    create_timer();

    RCLCPP_INFO(this->get_logger(), "LidarPublisher started, freq=%.2f Hz", frequency_);
  }

private:
  void create_timer(){
    if (frequency_ <= 0.0) {
      frequency_ = 1.0;
    }
    auto period = std::chrono::duration<double>(1.0 / frequency_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LidarPublisher::timer_callback, this));
  }

  void timer_callback(){
    robot_control::msg::LidarData msg;
    msg.stamp = this->now();

    static std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<float> dist(0.1f, 2.0f);

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      msg.ranges[i] = dist(gen);
    }

    pub_->publish(msg);
  }

  rcl_interfaces::msg::SetParametersResult
  on_param_set(const std::vector<rclcpp::Parameter> & params){
    for (const auto & p : params) {
      if (p.get_name() == "lidar_publish_frequency") {
        double new_freq = p.as_double();
        if (new_freq <= 0.0) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = false;
          res.reason = "lidar_publish_frequency must be > 0";
          return res;
        }
        frequency_ = new_freq;
        timer_->cancel();
        create_timer();
        RCLCPP_INFO(this->get_logger(), "Updated lidar_publish_frequency = %.2f", frequency_);
      }
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  double frequency_;
  rclcpp::Publisher<robot_control::msg::LidarData>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPublisher>());
  rclcpp::shutdown();
  return 0;
}
