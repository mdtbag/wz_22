#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std::chrono_literals;

struct Point2D {
    double x, y;
    Point2D(double x = 0.0, double y = 0.0) : x(x), y(y) {}
};

class RobotSensorNode : public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr observed_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Point2D> data_points_;
    size_t current_index_ = 0;
    std::string observation_frame_;
    bool loop_data_ = true;
public:
    RobotSensorNode() : Node("robot_sensor")
    {
        this->declare_parameter("frequency", 2.0);
        this->declare_parameter("data_file_path", "");
        this->declare_parameter("observation_frame", "base_link");
        this->declare_parameter("loop_data", true);

        double freq = this->get_parameter("frequency").as_double();
        std::string file_path = this->get_parameter("data_file_path").as_string();
        observation_frame_ = this->get_parameter("observation_frame").as_string();
        loop_data_ = this->get_parameter("loop_data").as_bool();

        if (file_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "data_file_path is empty! Please provide a valid path.");
            rclcpp::shutdown();
            return;
        }

        // 读取数据
        if (!loadData(file_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load data from %s", file_path.c_str());
            rclcpp::shutdown();
            return;
        }

        observed_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_sensor/observed", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 启动定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / freq)),
            std::bind(&RobotSensorNode::publishData, this)
        );

        RCLCPP_INFO(this->get_logger(), "RobotSensorNode started with frequency=%.1f Hz", freq);
    }

private:
    void publishData(){
        if (data_points_.empty()) return;

        // 获取当前点
        const auto& pt = data_points_[current_index_];

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = observation_frame_;
        msg.pose.position.x = pt.x;
        msg.pose.position.y = pt.y;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;
        observed_pub_->publish(msg);

        // 广播 TF: world -> base_link（固定）
        geometry_msgs::msg::TransformStamped world_to_base;
        world_to_base.header.stamp = this->now();
        world_to_base.header.frame_id = "world";
        world_to_base.child_frame_id = "base_link";
        world_to_base.transform.translation.x = 0.0;
        world_to_base.transform.translation.y = 0.0;
        world_to_base.transform.translation.z = 0.0;
        world_to_base.transform.rotation.x = 0.0;
        world_to_base.transform.rotation.y = 0.0;
        world_to_base.transform.rotation.z = 0.0;
        world_to_base.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(world_to_base);

        // 广播 TF: base_link -> target_link
        geometry_msgs::msg::TransformStamped base_to_target;
        base_to_target.header.stamp = this->now();
        base_to_target.header.frame_id = "base_link";
        base_to_target.child_frame_id = "target_link";
        base_to_target.transform.translation.x = pt.x;
        base_to_target.transform.translation.y = pt.y;
        base_to_target.transform.translation.z = 0.0;
        base_to_target.transform.rotation.x = 0.0;
        base_to_target.transform.rotation.y = 0.0;
        base_to_target.transform.rotation.z = 0.0;
        base_to_target.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(base_to_target);

        // 更新索引
        current_index_++;
        if (current_index_ >= data_points_.size()) {
            if (loop_data_) {
                current_index_ = 0;
            } else {
                timer_->cancel(); // 停止发布
                RCLCPP_INFO(this->get_logger(), "Finished playing data (loop_data=false).");
            }
        }
    }

    bool loadData(const std::string& filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", filepath.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            // 忽略空行和注释
            if (line.empty() || line[0] == '#') continue;
            std::istringstream iss(line);
            double x, y;
            if (iss >> x >> y) {
                data_points_.emplace_back(x, y);
            }
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded %zu data points from %s", data_points_.size(), filepath.c_str());
        return true;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
