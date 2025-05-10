#include "control_node.hpp"

namespace robot {

ControlNode::ControlNode() : Node("control_node") {
  double lookahead = 1.5;
  double tolerance = 0.5;
  double speed = 0.5;

  controller_ = std::make_shared<ControlCore>(lookahead, tolerance, speed);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", rclcpp::SensorDataQoS(), std::bind(&ControlNode::scanCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "ControlNode ready.");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  latest_path_ = *msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_pose_ = msg->pose.pose;
}

void ControlNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  latest_scan_ = msg;
}

void ControlNode::controlLoop() {
  if (latest_path_.poses.empty()) return;

  const auto& goal_pose = latest_path_.poses.back().pose;

  if (controller_->goalReached(latest_pose_, goal_pose)) {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_vel_pub_->publish(stop);
    return;
  }

  // Check for obstacles directly ahead
  bool obstacle_ahead = false;
  if (latest_scan_) {
    int start = latest_scan_->ranges.size() / 3;
    int end = 2 * latest_scan_->ranges.size() / 3;
    for (int i = start; i < end; ++i) {
      float range = latest_scan_->ranges[i];
      if (range < 0.5 && range > latest_scan_->range_min) {
        obstacle_ahead = true;
        break;
      }
    }
  }

  if (obstacle_ahead) {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_vel_pub_->publish(stop);
    return;
  }

  auto target = controller_->findTarget(latest_path_, latest_pose_);
  auto cmd = controller_->computeVelocity(latest_pose_, target);
  cmd_vel_pub_->publish(cmd);
}

}  // namespace robot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot::ControlNode>());
  rclcpp::shutdown();
  return 0;
}
