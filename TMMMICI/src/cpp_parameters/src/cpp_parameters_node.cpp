#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <functional>
#include <string>

using namespace std::chrono_literals;

class p : public rclcpp::Node{
public:
    p() : Node("sp"){
        this->declare_parameter("my_param","my_world");

        timer_ = this->create_wall_timer(1s,std::bind(&p::call_back,this));
    }
private:
    std::string my_param = "my_world";
    void call_back(){
        my_param = this->get_parameter("my_param").as_string();
        RCLCPP_INFO(this->get_logger(),"hello %s",my_param.c_str());

        std::vector<rclcpp::Parameter> all_new_params{rclcpp::Parameter("my_param",my_param)};
        this->set_parameters(all_new_params);
    }
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char ** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<p>());
    rclcpp::shutdown();

    return 0;
}