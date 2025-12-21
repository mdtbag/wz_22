#include "robot_simulator/simulator_node.hpp"
#include <cmath>
#include <tf2/exceptions.h>

namespace robot_simulator {

SimulatorNode::SimulatorNode() : Node("simulator_node") {
  // 声明参数
  this->declare_parameter("target_initial_x", 5.0);
  this->declare_parameter("target_initial_y", 1.0);
  this->declare_parameter("target_velocity_x", 1.0);
  this->declare_parameter("target_velocity_y", 0.5);
  this->declare_parameter("target_movement_range_x", 3.0);
  this->declare_parameter("target_movement_range_y", 4.0);
  this->declare_parameter("bullet_speed", 20.0);
  this->declare_parameter("hit_threshold", 0.25);
  this->declare_parameter("hit_count_threshold", 3);

  // 获取参数
  target_x_ = this->get_parameter("target_initial_x").as_double();
  target_y_ = this->get_parameter("target_initial_y").as_double();
  target_vx_ = this->get_parameter("target_velocity_x").as_double();
  target_vy_ = this->get_parameter("target_velocity_y").as_double();
  target_range_x_ = this->get_parameter("target_movement_range_x").as_double();
  target_range_y_ = this->get_parameter("target_movement_range_y").as_double();
  bullet_speed_ = this->get_parameter("bullet_speed").as_double();
  hit_threshold_ = this->get_parameter("hit_threshold").as_double();
  hit_count_threshold_ = this->get_parameter("hit_count_threshold").as_int();

  // 初始化 TF 广播器和监听器
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建 Marker 发布器
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization_markers", 10);

  // 创建 JointState 发布器
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // 订阅控制指令
  control_sub_ = this->create_subscription<robot_interfaces::msg::Control>(
      "robot_solver/control", 10,
      std::bind(&SimulatorNode::controlCallback, this, std::placeholders::_1));

  // 创建定时器 (100Hz)
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(10),
                              std::bind(&SimulatorNode::timerCallback, this));

  // 广播静态 TF: world -> base_link
  broadcastStaticBaseLink();

  RCLCPP_INFO(this->get_logger(), "Simulator node started");
}

void SimulatorNode::broadcastStaticBaseLink() {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "base_link";

  // 机器人位置固定在原点
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  static_tf_broadcaster_->sendTransform(transform);
}

void SimulatorNode::publishJointStates() {
  auto joint_msg = sensor_msgs::msg::JointState();
  joint_msg.header.stamp = this->now();
  joint_msg.name = {"base_to_gimbal"}; // 关节名称（从 URDF）
  joint_msg.position = {current_gimbal_yaw_ * M_PI / 180.0}; // 转换为弧度
  joint_msg.velocity = {0.0};
  joint_msg.effort = {0.0};

  joint_state_pub_->publish(joint_msg);
}

void SimulatorNode::broadcastTargetLink() {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "target_link";

  transform.transform.translation.x = target_x_;
  transform.transform.translation.y = target_y_;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(transform);
}

void SimulatorNode::broadcastBulletLink() {
  if (!bullet_active_)
    return;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "bullet_link";

  transform.transform.translation.x = bullet_x_;
  transform.transform.translation.y = bullet_y_;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(transform);
}

void SimulatorNode::updateTarget(double dt) {
  if (target_stopped_)
    return;

  // 更新目标位置
  target_x_ += target_vx_ * dt;
  target_y_ += target_vy_ * dt;

  // 边界反弹（以world为中心，范围为 ±target_range）
  if (target_x_ <= -target_range_x_ || target_x_ >= target_range_x_) {
    target_vx_ = -target_vx_;
    target_x_ = std::clamp(target_x_, -target_range_x_, target_range_x_);
  }
  if (target_y_ <= -target_range_y_ || target_y_ >= target_range_y_) {
    target_vy_ = -target_vy_;
    target_y_ = std::clamp(target_y_, -target_range_y_, target_range_y_);
  }
}

void SimulatorNode::updateBullet(double dt) {
  if (!bullet_active_)
    return;

  // 更新弹丸位置
  bullet_x_ += bullet_vx_ * dt;
  bullet_y_ += bullet_vy_ * dt;

  // 检测碰撞
  double distance = std::sqrt(std::pow(bullet_x_ - target_x_, 2) +
                              std::pow(bullet_y_ - target_y_, 2));

  if (distance < hit_threshold_) {
    // 击中目标
    hit_count_++;
    RCLCPP_INFO(this->get_logger(), "Target hit! Count: %d", hit_count_);
    bullet_active_ = false;

    // 多次击中后停止目标
    if (hit_count_ >= hit_count_threshold_) {
      target_stopped_ = true;
      RCLCPP_INFO(this->get_logger(), "Target stopped after %d hits",
                  hit_count_);
    }
  }

  // 弹丸飞出边界，销毁
  if (bullet_x_ <= -target_range_x_ || bullet_x_ >= target_range_x_ ||
      bullet_y_ <= -target_range_y_ || bullet_y_ >= target_range_y_) {
    bullet_active_ = false;
    RCLCPP_DEBUG(this->get_logger(), "Bullet out of bounds");
  }
}

void SimulatorNode::controlCallback(
    const robot_interfaces::msg::Control::SharedPtr msg) {

  // 根据击打模式更新云台角度
  if (msg->hit_mode == "positive") {
    // Positive 模式：云台跟随目标，使用消息中的 yaw_angle
    current_gimbal_yaw_ = msg->yaw_angle;
  } else if (msg->hit_mode == "passive") {
    // Passive 模式：云台固定朝向（yaw = 0）
    current_gimbal_yaw_ = 0.0;
  }

  // 发布关节状态（更新云台角度）
  publishJointStates();

  // 收到开火建议
  if (msg->fire && !bullet_active_) {
    // 通过 TF 获取 gimbal_link 在 world 坐标系下的位置
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_->lookupTransform("world", "gimbal_link",
                                                      tf2::TimePointZero);

      // 弹丸初始位置 = gimbal_link 在 world 坐标系下的位置
      bullet_x_ = transform_stamped.transform.translation.x;
      bullet_y_ = transform_stamped.transform.translation.y;

      // 弹丸速度方向 = 云台朝向（使用当前云台角度）
      double yaw_rad = current_gimbal_yaw_ * M_PI / 180.0;
      bullet_vx_ = bullet_speed_ * std::cos(yaw_rad);
      bullet_vy_ = bullet_speed_ * std::sin(yaw_rad);

      bullet_active_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Bullet fired from (%.2f, %.2f) at angle: %.2f degrees "
                  "(mode: %s)",
                  bullet_x_, bullet_y_, current_gimbal_yaw_,
                  msg->hit_mode.c_str());
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get gimbal_link transform: %s",
                  ex.what());
    }
  }
}

void SimulatorNode::timerCallback() {
  double dt = 0.01; // 100Hz -> 0.01s

  // 更新目标和弹丸
  updateTarget(dt);
  updateBullet(dt);

  // 发布关节状态（持续更新云台角度）
  publishJointStates();

  // 广播 TF
  broadcastTargetLink();
  broadcastBulletLink();

  // 发布 Markers
  publishMarkers();
}

void SimulatorNode::publishMarkers() {
  visualization_msgs::msg::MarkerArray marker_array;

  // 运动边界 Marker (白色线框)
  visualization_msgs::msg::Marker boundary_marker;
  boundary_marker.header.frame_id = "world";
  boundary_marker.header.stamp = this->now();
  boundary_marker.ns = "simulator";
  boundary_marker.id = 3;
  boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  boundary_marker.action = visualization_msgs::msg::Marker::ADD;
  boundary_marker.pose.orientation.w = 1.0;
  boundary_marker.scale.x = 0.05; // 线宽
  boundary_marker.color.r = 1.0;
  boundary_marker.color.g = 1.0;
  boundary_marker.color.b = 1.0;
  boundary_marker.color.a = 0.8;

  // 绘制矩形边界 (以world为中心，z=0)
  geometry_msgs::msg::Point p;
  // 左下角
  p.x = -target_range_x_;
  p.y = -target_range_y_;
  p.z = 0.0;
  boundary_marker.points.push_back(p);
  // 右下角
  p.x = target_range_x_;
  p.y = -target_range_y_;
  p.z = 0.0;
  boundary_marker.points.push_back(p);
  // 右上角
  p.x = target_range_x_;
  p.y = target_range_y_;
  p.z = 0.0;
  boundary_marker.points.push_back(p);
  // 左上角
  p.x = -target_range_x_;
  p.y = target_range_y_;
  p.z = 0.0;
  boundary_marker.points.push_back(p);
  // 回到左下角闭合
  p.x = -target_range_x_;
  p.y = -target_range_y_;
  p.z = 0.0;
  boundary_marker.points.push_back(p);

  marker_array.markers.push_back(boundary_marker);

  // 目标 Marker (红色球体)
  visualization_msgs::msg::Marker target_marker;
  target_marker.header.frame_id = "target_link";
  target_marker.header.stamp = this->now();
  target_marker.ns = "simulator";
  target_marker.id = 0;
  target_marker.type = visualization_msgs::msg::Marker::SPHERE;
  target_marker.action = visualization_msgs::msg::Marker::ADD;
  target_marker.pose.position.x = 0.0;
  target_marker.pose.position.y = 0.0;
  target_marker.pose.position.z = 0.24;
  target_marker.pose.orientation.w = 1.0;
  target_marker.scale.x = 0.5; // 直径 = 半径 * 2 = 0.25 * 2
  target_marker.scale.y = 0.5;
  target_marker.scale.z = 0.5;
  target_marker.color.r = 0.8;
  target_marker.color.g = 0.2;
  target_marker.color.b = 0.2;
  target_marker.color.a = 1.0;
  marker_array.markers.push_back(target_marker);

  // 弹丸 Marker (绿色球体)
  if (bullet_active_) {
    visualization_msgs::msg::Marker bullet_marker;
    bullet_marker.header.frame_id = "bullet_link";
    bullet_marker.header.stamp = this->now();
    bullet_marker.ns = "simulator";
    bullet_marker.id = 1;
    bullet_marker.type = visualization_msgs::msg::Marker::SPHERE;
    bullet_marker.action = visualization_msgs::msg::Marker::ADD;
    bullet_marker.pose.position.x = 0.0;
    bullet_marker.pose.position.y = 0.0;
    bullet_marker.pose.position.z = 0.0;
    bullet_marker.pose.orientation.w = 1.0;
    bullet_marker.scale.x = 0.1; // 直径 = 0.05 * 2
    bullet_marker.scale.y = 0.1;
    bullet_marker.scale.z = 0.1;
    bullet_marker.color.r = 0.2;
    bullet_marker.color.g = 0.8;
    bullet_marker.color.b = 0.2;
    bullet_marker.color.a = 1.0;
    marker_array.markers.push_back(bullet_marker);
  } else {
    // 删除弹丸 Marker
    visualization_msgs::msg::Marker bullet_marker;
    bullet_marker.header.frame_id = "world";
    bullet_marker.header.stamp = this->now();
    bullet_marker.ns = "simulator";
    bullet_marker.id = 1;
    bullet_marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_array.markers.push_back(bullet_marker);
  }

  marker_pub_->publish(marker_array);
}

} // namespace robot_simulator

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_simulator::SimulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
