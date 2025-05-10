#include "costmap_node.hpp"

CostmapNode::CostmapNode()
  : Node("costmap_node"),
    costmap_core_(robot::CostmapCore(this->get_logger())) 
{
  // Set basic parameters
  double resolution = 0.1;
  int width = 300;
  int height = 300;
  double inflation_radius = 1.5;

  geometry_msgs::msg::Pose origin;
  origin.position.x = -15.0;
  origin.position.y = -15.0;
  origin.position.z = 0.0;
  origin.orientation.w = 1.0;

  // Initialize costmap
  costmap_core_.initCostmap(costmap_, resolution, width, height, origin, inflation_radius);

  // Publisher for the costmap
  costmap_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // Subscriber for laser scan data
  laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::handleLaserScan, this, std::placeholders::_1));
}

void CostmapNode::handleLaserScan(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_core_.updateCostmap(costmap_, msg);

  costmap_.header.stamp = msg->header.stamp;
  costmap_.header.frame_id = "robot/chassis/lidar";

  costmap_publisher_->publish(costmap_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
