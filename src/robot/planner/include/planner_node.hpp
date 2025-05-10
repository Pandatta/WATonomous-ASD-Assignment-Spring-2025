#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

// This class handles path planning from current pose to goal using map info
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  // Simple state machine to track goal state
  enum class State {
    WAITING_FOR_GOAL,
    TRACKING_GOAL
  };

  State state_;

  // Subscriptions and publisher
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timer for periodic path planning
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest data
  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose current_pose_;
  bool goal_received_ = false;

  // Callbacks
  void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // Helpers
  void planPath();
  bool isGoalReached(double tolerance = 0.5);
};

#endif  // PLANNER_NODE_HPP_
