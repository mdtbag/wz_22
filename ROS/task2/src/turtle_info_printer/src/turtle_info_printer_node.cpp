#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class PosePrinter : public rclcpp::Node {
private:
void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) const {
  RCLCPP_INFO(this->get_logger(),
              "x=%.2f y=%.2f theta=%.2f lin=%.2f ang=%.2f",
              msg->x, msg->y, msg->theta,
              msg->linear_velocity, msg->angular_velocity);
  }
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
public:
  PosePrinter() : Node("turtle_info_printer") {
    using std::placeholders::_1;
    sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",rclcpp::QoS(10),std::bind(&PosePrinter::pose_callback, this, _1));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePrinter>());
  rclcpp::shutdown();
  return 0;
}