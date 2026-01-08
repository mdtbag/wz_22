#include "rclcpp/rclcpp.hpp"
#include "hahaa/msg/so.hpp"
#include "hahaa/msg/wts.hpp"

#include <memory>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class So : public rclcpp::Node{
    public:
    So() : Node("So"){
        lis_ = this->create_subscription<hahaa::msg::Wts>("xxx",10,std::bind(&So::listen,this,std::placeholders::_1));
        So_ = this->create_publisher<hahaa::msg::So>("xxxxx",10);
    }
    private:
    void listen(hahaa::msg::Wts::SharedPtr com){
        auto_pub(com->command);
    }
    void auto_pub(const std::string& co){
        auto ans = hahaa::msg::So();
        ans.words = "？？？";
        this->So_->publish(ans);
    }
    rclcpp::Publisher<hahaa::msg::So>::SharedPtr So_;
    rclcpp::Subscription<hahaa::msg::Wts>::SharedPtr lis_;
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<So>());
    rclcpp::shutdown();

    return 0;
}
