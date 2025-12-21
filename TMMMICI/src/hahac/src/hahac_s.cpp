#include "rclcpp/rclcpp.hpp"
#include "hahac/action/wts.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <thread>

using namespace std::chrono_literals;
using local = hahac::action::Wts;

void say(const char* s){
    RCLCPP_INFO(rclcpp::get_logger("So"),"%s",s);
}

class So : public rclcpp::Node{
    public:
    So() : Node("So"){
        na = this->get_name();
        auto hr = std::bind(&So::h_r,this,std::placeholders::_1,std::placeholders::_2);
        auto hj = std::bind(&So::h_j,this,std::placeholders::_1);
        auto ha = std::bind(&So::h_a,this,std::placeholders::_1);
        lis_ = rclcpp_action::create_server<hahac::action::Wts>(this,"cheer",hr,hj,ha);
    }

    private:

    rclcpp_action::GoalResponse h_r(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const hahac::action::Wts::Goal> goal){
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse h_j(std::shared_ptr<rclcpp_action::ServerGoalHandle<hahac::action::Wts>> gh){
        RCLCPP_WARN(this->get_logger(),"have canceled %s",gh->get_goal()->command.c_str());
        return rclcpp_action::CancelResponse::REJECT;
    }

    void h_a(std::shared_ptr<rclcpp_action::ServerGoalHandle<hahac::action::Wts>> gh){
        std::thread{std::bind(&So::exe,this,gh)}.detach();
    }

    void exe(std::shared_ptr<rclcpp_action::ServerGoalHandle<hahac::action::Wts>> gh){
        std::string goal = gh->get_goal()->command;
        auto feedback = std::make_shared<hahac::action::Wts::Feedback>();
        auto result = std::make_shared<local::Result>();

        rclcpp::Rate rate(1s);
        for(size_t i = 1;i <= goal.size();i++){
            if(gh->is_canceling()){
                result->ans = "canceled";
                gh->canceled(result);
                say(result->ans.c_str());
                return;
            }
            feedback->wo = ans1.substr(0,i-1);
            gh->publish_feedback(feedback);
            say(feedback->wo.c_str());
            rate.sleep();
        }

        if(rclcpp::ok() && !gh->is_canceling()){
            result->ans = feedback->wo + "!!!";
            gh->succeed(result);
            say(result->ans.c_str());
        }
    }

    rclcpp_action::Server<hahac::action::Wts>::SharedPtr lis_;  
    std::string na;
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor ex;
    auto node = std::make_shared<So>();
    ex.add_node(node);
    ex.spin();

    rclcpp::shutdown();

    return 0;
}
