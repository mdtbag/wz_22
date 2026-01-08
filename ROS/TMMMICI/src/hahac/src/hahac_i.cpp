#include "rclcpp/rclcpp.hpp"
#include "hahac/action/wts.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;
using r = rclcpp_action::ClientGoalHandle<hahac::action::Wts>;

class sir : public rclcpp::Node{
    public:
    sir() : Node("sir"){
        sir_ = rclcpp_action::create_client<hahac::action::Wts>(this,"cheer");
        timer_ = this->create_wall_timer(20s,std::bind(&sir::auto_pub,this));
        na = this->get_name();
    }
    private:
    
    void auto_pub(){
        auto words = std::make_shared<hahac::action::Wts::Goal>();
        RCLCPP_WARN(this->get_logger(),"you have only 20s to complete the action");
        if(!sir_->wait_for_action_server(5s)){
            RCLCPP_WARN(rclcpp::get_logger(na),"no server,skip");
            return;
        }
        
        words->command = "同志们辛苦了";
     rclcpp_action::Client<hahac::action::Wts>::SendGoalOptions options;
        options.goal_response_callback = std::bind(&sir::gc,this,std::placeholders::_1);
        options.feedback_callback = std::bind(&sir::fh,this,std::placeholders::_1,std::placeholders::_2);
        options.result_callback = std::bind(&sir::rc,this,std::placeholders::_1);
        sir_->async_send_goal(*words,options);
        RCLCPP_INFO(this->get_logger(),words->command.c_str());
    }

    void gc(r::SharedPtr fu){
        if(!fu.get()){
            RCLCPP_WARN(this->get_logger(),"goal refused");
            return;
        }
    }

    void fh(r::SharedPtr,const std::shared_ptr<const hahac::action::Wts::Feedback> feedback){
        return;
    }

    void rc(const rclcpp_action::ClientGoalHandle<hahac::action::Wts>::WrappedResult & result){
        return;
    }

    rclcpp_action::Client<hahac::action::Wts>::SharedPtr sir_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string na;
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<sir>());
    rclcpp::shutdown();

    return 0;
}
