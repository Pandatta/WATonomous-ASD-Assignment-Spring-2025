#include "planner_node.hpp"
#include "planner_core.hpp"
#include <cmath>

PlannerNode::PlannerNode()
  : Node("planner_node"), state_(State::WAITING_FOR_GOAL)
{
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_ = *msg;
}

void PlannerNode::goalCallback(geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::TRACKING_GOAL;
  planPath();
}

void PlannerNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ != State::TRACKING_GOAL || !goal_received_) return;

  if (isGoalReached()) {
    state_ = State::WAITING_FOR_GOAL;
  } else {
    planPath();
  }
}

bool PlannerNode::isGoalReached(double tol) {
  double dx = goal_.point.x - current_pose_.position.x;
  double dy = goal_.point.y - current_pose_.position.y;
  return std::hypot(dx, dy) < tol;
}

void PlannerNode::planPath() {
  if (map_.data.empty() || !goal_received_) return;

  auto path = PlannerCore::aStarPlan(map_, current_pose_, goal_.point);
  path.header.stamp = now();
  path.header.frame_id = "map";
  path_pub_->publish(path);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
