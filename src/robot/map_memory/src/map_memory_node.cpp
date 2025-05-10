#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory_node"),
    core_(this->get_logger()) {

  // Subscribe to costmap updates
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  // Subscribe to robot's current pose
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publish the fused global map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Set up a timer to publish the map regularly
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::publishMap, this));

  // Set up global map dimensions and origin
  global_map_.info.resolution = 0.1;
  global_map_.info.width = 300;
  global_map_.info.height = 300;
  global_map_.info.origin.position.x = -15.0;
  global_map_.info.origin.position.y = -15.0;
  global_map_.data.resize(300 * 300, -1);  // -1 means unknown
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = msg->pose.pose;
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  core_.fuseCostmapIntoMap(global_map_, *msg, current_pose_);
  costmap_received_ = true;
}

void MapMemoryNode::publishMap() {
  if (!costmap_received_) return;

  global_map_.header.stamp = this->get_clock()->now();
  global_map_.header.frame_id = "map";
  map_pub_->publish(global_map_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
