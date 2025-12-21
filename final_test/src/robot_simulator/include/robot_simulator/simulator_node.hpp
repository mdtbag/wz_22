#ifndef ROBOT_SIMULATOR__SIMULATOR_NODE_HPP_
#define ROBOT_SIMULATOR__SIMULATOR_NODE_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/control.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace robot_simulator {

class SimulatorNode : public rclcpp::Node {
public:
  SimulatorNode();

private:
  // TF 广播函数
  void broadcastStaticBaseLink();
  void publishJointStates(); // 发布关节状态
  void broadcastTargetLink();
  void broadcastBulletLink();

  // 更新函数
  void updateTarget(double dt);
  void updateBullet(double dt);

  // 回调函数
  void controlCallback(const robot_interfaces::msg::Control::SharedPtr msg);
  void timerCallback();

  // 可视化函数
  void publishMarkers();

  // 参数
  double target_x_, target_y_;
  double target_vx_, target_vy_;
  double target_range_x_, target_range_y_;
  double bullet_speed_;
  double hit_threshold_;
  int hit_count_threshold_;

  // 状态
  bool target_stopped_ = false;
  int hit_count_ = 0;
  double current_gimbal_yaw_ = 0.0; // 当前云台角度（度）

  // 弹丸状态
  bool bullet_active_ = false;
  double bullet_x_, bullet_y_;
  double bullet_vx_, bullet_vy_;

  // ROS 组件
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<robot_interfaces::msg::Control>::SharedPtr control_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace robot_simulator

#endif // ROBOT_SIMULATOR__SIMULATOR_NODE_HPP_
