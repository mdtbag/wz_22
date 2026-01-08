#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <chrono>
#include "std_srvs/srv/empty.hpp" 

using namespace std;
using namespace std::chrono_literals;

class TurtleSquareNode : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; 
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr Subscriber_; 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    double length = 1.0;
    double mv_ang = 0.0;
    double current_ang = 0.0;
public:
    double start_x, start_y;   
    double target_angle = 2*M_PI;  
    TurtleSquareNode(const string &node_name) : Node(node_name){
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/turtlesim");
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        Subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, 
                    bind(&TurtleSquareNode::draw_triangle, this, placeholders::_1));
        this->declare_parameter("length", 1.0);
        length = this->get_parameter("length").as_double();
    }

    double stdan(double an){
        while(an < 0){
            an += 2.0 * M_PI;
        }
        return  an;
    }

    void draw_triangle(const turtlesim::msg::Pose::SharedPtr pos) {

        auto msg = geometry_msgs::msg::Twist();

        if (pos->x == 0.0 && pos->y == 0.0 && this->mv_ang >= 0.0) {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "Done!");
            rclcpp::shutdown();
        }
        mv_ang += stdan(pos->theta) - stdan(current_ang);
        double angle_diff = this->target_angle - mv_ang;
        current_ang = pos->theta;
        msg.angular.z = 1.1 * angle_diff;
        msg.linear.x = 1.1 * angle_diff * length;

        publisher_->publish(msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtleSquareNode>("turtle_square");
    rclcpp::spin(node);
    return 0;
}
