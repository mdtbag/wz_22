#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class sfp : public rclcpp::Node{
    public:
    sfp(char * tr[]) : Node("sfp"){
        tsb_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->make_transforms(tr);
    }
    private:
    void make_transforms(char * tr[]){
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_logger()->now;
        t.header.frame_id = "world";
        t.child_frame_id = tr[1];

        t.transform.translation.x = atof(tr[2]);
        t.transform.translation.y = atof(tr[3]);
        t.transform.translation.z = atof(tr[4]);
        tf2::Quaternion q;
        q.setRPY(
            atof(tr[5]),
            atof(tr[6]),
            atof(tr[7])
        )
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.y = q.z();
        t.transform.rotation.y = q.w();

        tsb_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tab_;
};

int main(int argc,char** argv){
    auto logger = rclcpp::get_logger("logger");

    if(argc != 8){
        RCLCPP_INFO(
            logger,
            "invalid number ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw"
        );
        return 1;
    }

    if(strcmp(argv[1],"world") == 0){
        RCLCPP_INFO(logger,"can not");
    }

    rclcpp::init(argc,argv);
    rclcpp::spin(std::shared_ptr<sfp>(argv));
    rclcpp::shutdown();

    return 0;
}