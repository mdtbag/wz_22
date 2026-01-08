#include <chrono>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"
#include "robot_control/msg/lidar_data.hpp"

using namespace std::chrono_literals;

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector() : Node("obstacle_detector"), has_data_(false)
  {
    threshold_ = this->declare_parameter<double>("obstacle_threshold", 0.5);

    sub_ = this->create_subscription<robot_control::msg::LidarData>(
      "lidar_data", 10,
      std::bind(&ObstacleDetector::lidar_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<std_msgs::msg::String>("obstacle_warning", 10);

    timer_ = this->create_wall_timer(
      200ms,
      std::bind(&ObstacleDetector::timer_callback, this));

    param_cb_ = this->add_on_set_parameters_callback(
      std::bind(&ObstacleDetector::on_param_set, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ObstacleDetector started, threshold=%.2f", threshold_);
  }

private:
  void lidar_callback(const robot_control::msg::LidarData::SharedPtr msg)
  {
    last_scan_ = *msg;
    has_data_ = true;
  }

  void timer_callback()
  {
    size_t min_index;
    
    if (!has_data_) return;
    bool flag=false;
    float min_d = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < last_scan_.ranges.size(); ++i) {
	  float d = last_scan_.ranges[i]; 
	  if (d < min_d) {
	    min_d = d;
	    min_index = i;
	  }
    }
    std_msgs::msg::String out;
    if (min_d < threshold_) {
      out.data = "Obstacle detected!" + std::to_string(min_index*36) + " degrees";
      flag = 1;
    } 
    else {
    	out.data = "Obstacle not found,the nearest one : " + std::to_string(min_d);
    }
    pub_->publish(out);
    if(flag == 0) RCLCPP_INFO(get_logger(),"%s",out.data.c_str());
    else RCLCPP_FATAL(get_logger(),"%s",out.data.c_str());
  }
   

  rcl_interfaces::msg::SetParametersResult
  on_param_set(const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      if (p.get_name() == "obstacle_threshold") {
        double t = p.as_double();
        if (t <= 0.0) {
          rcl_interfaces::msg::SetParametersResult res;
          res.successful = false;
          res.reason = "obstacle_threshold must be > 0";
          return res;
        }
        threshold_ = t;
        RCLCPP_INFO(this->get_logger(), "Updated obstacle_threshold = %.2f", threshold_);
      }
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
  }

  double threshold_;
  bool has_data_;
  robot_control::msg::LidarData last_scan_;

  rclcpp::Subscription<robot_control::msg::LidarData>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
