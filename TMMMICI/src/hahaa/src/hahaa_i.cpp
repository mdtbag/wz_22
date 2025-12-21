#include "rclcpp/rclcpp.hpp"
#include "hahaa/msg/wts.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class sir : public rclcpp::Node{
    public:
    sir() : Node("sir"){
        sir_ = this->create_publisher<hahaa::msg::Wts>("xxx",10);
        timer_ = this->create_wall_timer(10s,std::bind(&sir::auto_pub,this)); 
    }
    private:
    void auto_pub(){
        auto words =  hahaa::msg::Wts();
				words.command = "xxx";
        words.cnt = xxx;
        this->sir_->publish(words);
    }
    rclcpp::Publisher<hahaa::msg::Wts>::SharedPtr sir_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<sir>());
    rclcpp::shutdown();

    return 0;
}
