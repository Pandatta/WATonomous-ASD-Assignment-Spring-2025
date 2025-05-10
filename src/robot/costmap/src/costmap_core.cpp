#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger), inflation_radius_(1.0), resolution_(0.1) {}

void CostmapCore::initCostmap(
    nav_msgs::msg::OccupancyGrid& costmap,
    double resolution,
    int width,
    int height,
    const geometry_msgs::msg::Pose& origin,
    double inflation_radius)
{
  resolution_ = resolution;
  inflation_radius_ = inflation_radius;

  costmap.info.resolution = resolution;
  costmap.info.width = width;
  costmap.info.height = height;
  costmap.info.origin = origin;

  costmap.data.assign(width * height, 0);  // start with free space
}

void CostmapCore::updateCostmap(
    nav_msgs::msg::OccupancyGrid& costmap,
    const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
  std::fill(costmap.data.begin(), costmap.data.end(), 0);

  double angle = scan->angle_min;

  for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
    double range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max) continue;

    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int gx = static_cast<int>((x - costmap.info.origin.position.x) / resolution_);
    int gy = static_cast<int>((y - costmap.info.origin.position.y) / resolution_);

    if (gx < 0 || gx >= static_cast<int>(costmap.info.width) ||
        gy < 0 || gy >= static_cast<int>(costmap.info.height)) continue;

    int idx = gy * costmap.info.width + gx;
    costmap.data[idx] = 100;

    inflateObstacle(costmap, gx, gy);
  }
}

void CostmapCore::inflateObstacle(nav_msgs::msg::OccupancyGrid& costmap, int ox, int oy)
{
  int width = costmap.info.width;
  int height = costmap.info.height;
  int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
  int max_cost = 100;

  for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      int nx = ox + dx;
      int ny = oy + dy;

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

      double distance = std::hypot(dx, dy) * resolution_;
      if (distance > inflation_radius_) continue;

      int cost = static_cast<int>(max_cost * (1.0 - (distance / inflation_radius_)));
      int idx = ny * width + nx;

      costmap.data[idx] = std::max(costmap.data[idx], static_cast<int8_t>(cost));
    }
  }
}

}  // namespace robot
