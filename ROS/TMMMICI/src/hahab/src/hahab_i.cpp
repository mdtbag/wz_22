#include "rclcpp/rclcpp.hpp"
#include "hahab/srv/wts.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class sir : public rclcpp::Node{
    public:
    
    sir() : Node("sir"){
        sir_ = this->create_client<hahab::srv::Wts>("cheer");
        timer_ = this->create_wall_timer(2s,std::bind(&sir::auto_pub,this));
    }
    
    private:
    
    void auto_pub(){
        if(!sir_->wait_for_service(1s)){
            RCLCPP_WARN(rclcpp::get_logger(na),"no response,skip");
            return;
        }
        
        auto words =  std::make_shared<hahab::srv::Wts::Request>();
        words->command = "xxx";
        sir_->async_send_request(words,std::bind(&sir::re,this,std::placeholders::_1));
    }
    
    void re(const rclcpp::Client<hahab::srv::Wts>::SharedFuture fu){
        auto response = fu.get();
        RCLCPP_WARN(this->get_logger(),response->ans.c_str());
    }
    
    rclcpp::Client<hahab::srv::Wts>::SharedPtr sir_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<sir>());
    rclcpp::shutdown();

    return 0;
}
