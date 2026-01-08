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
    double pre_x = 5.0;
    double pre_y = 5.0;
    double mv_length = 0.0;
    double pre_ang = 0.0;
    double ro_ang = 0.0;
public:
    int side_count = 0;  
    bool first_call = true; 
    bool turning = false; 
    double start_x, start_y;   
    double target_angle;  
    int eg;  
    TurtleSquareNode(const string &node_name) : Node(node_name){
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/turtlesim");
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        Subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, 
                    bind(&TurtleSquareNode::draw_triangle, this, placeholders::_1));
        timer_=this->create_wall_timer(1s,std::bind(&TurtleSquareNode::change_turtlesim_background,this));
        this->declare_parameter("eg", 4);
        eg = this->get_parameter("eg").as_int();
        this->declare_parameter("length", 1.0);
        length = this->get_parameter("length").as_double();
    }
    
    int r = 200, g = 140, b = 20;

    void change_turtlesim_background(){

        r = (r + 60) % 256;
        g = (g + 60) % 256;
        b = (b + 60) % 256;

        param_client_->set_parameters({
            rclcpp::Parameter("background_r", r),
            rclcpp::Parameter("background_g", g),
            rclcpp::Parameter("background_b", b)
        });

        auto clear_client =
            this->create_client<std_srvs::srv::Empty>("/turtlesim/clear");

        if (clear_client->wait_for_service(std::chrono::milliseconds(100))) {
            auto req = std::make_shared<std_srvs::srv::Empty::Request>();
            clear_client->async_send_request(req);
        }
    }

    void stdan(double an){
        while(an < 0){
            an += 2.0 * M_PI;
        }
    }

    void draw_triangle(const turtlesim::msg::Pose::SharedPtr pos) {
        if (first_call) {
            start_x = pos->x;
            start_y = pos->y;
            first_call = false;
        }

        double current_x = pos->x;
        double current_y = pos->y;
        double current_theta = pos->theta;
        stdan(current_theta);

        auto msg = geometry_msgs::msg::Twist();

        if (side_count >= eg) {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
            RCLCPP_INFO(get_logger(), "Done!");
            rclcpp::shutdown();
        }

        if (!this->turning) {

            double dx = current_x - this->start_x;
            double dy = current_y - this->start_y;
            double distance = std::sqrt(dx*dx + dy*dy);

            if (distance >= this->length-0.05) {
                this->turning = true;
                this->side_count++;
                this->target_angle = current_theta + 2.0 * M_PI / eg;
                RCLCPP_INFO(get_logger(), "Turning to %.2f rad (side %d)", this->target_angle, this->side_count);
            } 
            else {
                msg.linear.x = 1.1 * (this->length-distance);
                msg.angular.z = 0.0;
            }
        } 
        else {
            double angle_diff = this->target_angle - current_theta;
            while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

            if (std::abs(angle_diff) < 0.05) {
                this->turning = false;
                this->start_x = current_x;
                this->start_y = current_y;
                RCLCPP_INFO(get_logger(), "Finished turn, moving straight (side %d)", this->side_count + 1);
            } else {
                msg.angular.z = 1.1 * angle_diff;
                msg.linear.x = 0.0;
            }
        }

        publisher_->publish(msg);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtleSquareNode>("turtle_square");
    rclcpp::spin(node);
    return 0;
}
