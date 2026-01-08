#include "rclcpp/rclcpp.hpp"
#include "hahab/srv/wts.hpp"

#include <memory>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class So : public rclcpp::Node{
    public:
    So() : Node("So"){
        na = this->get_name();
        lis_ = this->create_service<hahab::srv::Wts>("cheer",std::bind(&So::listen,this,std::placeholders::_1,std::placeholders::_2));
    }
    private:
    void listen(const std::shared_ptr<hahab::srv::Wts::Request>req,std::shared_ptr<hahab::srv::Wts::Response>res){
        auto co = req->command;
        if(co == "同志们好"){
            res->ans = na + "主席好";
            RCLCPP_FATAL(rclcpp::get_logger(na),na.c_str());
        }
        else{res->ans = na + "为人民服务";
        RCLCPP_FATAL(rclcpp::get_logger(na),na.c_str());}
    }
    rclcpp::Service<hahab::srv::Wts>::SharedPtr lis_;
    std::string na;
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<So>());
    rclcpp::shutdown();

    return 0;
}
