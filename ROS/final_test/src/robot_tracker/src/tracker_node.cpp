#include<cmath>
#include<memory>
#include<string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "robot_interfaces/action/collect.hpp"
#include "robot_interfaces/msg/target.hpp"

using Collect = robot_interfaces::action::Collect;
class TrackerNode : public rclcpp::Node{
    public:
    TrackerNode() : Node("robot_tracker"),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_),steady_clock_(RCL_STEADY_TIME){
        this->declare_parameter<double>("frequency",100.0);
        this->declare_parameter<double>("dt",0.01);
        this->declare_parameter<double>("static_velocity_threshold",0.1);
        this->declare_parameter<double>("static_time_threshold",2.0);

        frequency_=this->get_parameter("frequency").as_double();
        dt_fallback_=this->get_parameter("dt").as_double();
        static_v_th_=this->get_parameter("static_velocity_threshold").as_double();
        static_t_th_=this->get_parameter("static_time_threshold").as_double();

        pub_=this->create_publisher<robot_interfaces::msg::Target>("robot_tracker/target",10);
        action_client_=rclcpp_action::create_client<Collect>(this,"robot_controller/collect");

        auto period=std::chrono::duration<double>(1.0/std::max(1.0,frequency_));
        timer_=this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),std::bind(&TrackerNode::onTimer,this));
        RCLCPP_INFO(this->get_logger(),"robot_tracker started");
    }
    private:
    bool lookupXY(const std::string & target_frame,const std::string & source_frame,double & x,double & y){
        try{
            auto tf=tf_buffer_.lookupTransform(target_frame,source_frame,tf2::TimePointZero);
            x=tf.transform.translation.x;
            y=tf.transform.translation.y;
            return true;
        }catch(const std::exception &){
            return false;
        }
    }

    void onTimer(){
        rclcpp::Time now = this->now();
        double x=0.0,y=0.0;
        if(!lookupXY("gimbal_link","target_link",x,y)){
            return;
        }
        if(!has_prev_){
            prev_x_=x;
            prev_y_=y;
            prev_t_=now;
            has_prev_=true;
            return;
        }

        double dt=(now-prev_t_).seconds();
        if(dt<=1e-6) dt=dt_fallback_;

        double vx=(x-prev_x_)/dt;
        double vy=(y-prev_y_)/dt;

        prev_x_=x;
        prev_y_=y;
        prev_t_=now;

        robot_interfaces::msg::Target msg;
        msg.x=x;msg.y=y;msg.vx=vx;msg.vy=vy;
        pub_->publish(msg);

        double speed=std::hypot(vx,vy);
        double tick=1.0/std::max(1.0,frequency_);
        if(speed < static_v_th_) static_accum_+=tick;
        else static_accum_=0.0;

        if(!collecting_ && static_accum_>=static_t_th_){
            startCollect();
        }
    }
    void startCollect(){
        collecting_=true;
        static_accum_=0.0;

        if(!action_client_->wait_for_action_server(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(),"collect action server not available");
            collecting_=false;
            return;
        }

        double tx=0.0,ty=0.0;
        if(!lookupXY("world","target_link",tx,ty)){
            RCLCPP_WARN(this->get_logger(),"cannot lookup world->target_link for collect goal");
            collecting_=false;
            return;
        }

        Collect::Goal goal;
        goal.target_x=tx;
        goal.target_y=ty;

        RCLCPP_INFO(this->get_logger(),"send collect goal (%.2f,%.2f)",tx,ty);

        auto send_opts=rclcpp_action::Client<Collect>::SendGoalOptions();
        send_opts.result_callback=[this](const auto & result){
            if(result.code == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(this->get_logger(),"collect success=%d msg=%s",result.result->success,result.result->message.c_str());
            }
            else{
                RCLCPP_WARN(this->get_logger(),"collect finished with code %d",(int)result.code);
            }
            collecting_=false;
        };
        action_client_->async_send_goal(goal,send_opts);
    }
    private:
    rclcpp::Publisher<robot_interfaces::msg::Target>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Clock steady_clock_;

    rclcpp_action::Client<Collect>::SharedPtr action_client_;

    double frequency_{100.0};
    double dt_fallback_{0.01};
    double static_v_th_{0.1};
    double static_t_th_{2.0};

    bool has_prev_{false};
    double prev_x_{0.0},prev_y_{0.0};
    rclcpp::Time prev_t_{0,0,RCL_STEADY_TIME};

    double static_accum_{0.0};
    bool collecting_{false};
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TrackerNode>());
    rclcpp::shutdown();

    return 0;
}