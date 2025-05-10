#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::fuseCostmapIntoMap(nav_msgs::msg::OccupancyGrid &global_map,
                                       const nav_msgs::msg::OccupancyGrid &local_costmap,
                                       const geometry_msgs::msg::Pose &robot_pose) {
  // Robot's position in global map coordinates (in cells)
  int robot_x = static_cast<int>(
    (robot_pose.position.x - global_map.info.origin.position.x) / global_map.info.resolution);
  int robot_y = static_cast<int>(
    (robot_pose.position.y - global_map.info.origin.position.y) / global_map.info.resolution);

  // Half width/height of local costmap
  int half_w = local_costmap.info.width / 2;
  int half_h = local_costmap.info.height / 2;

  for (int y = 0; y < static_cast<int>(local_costmap.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(local_costmap.info.width); ++x) {

      // Compute target location in global map
      int global_x = robot_x - half_w + x;
      int global_y = robot_y - half_h + y;

      // Skip if out of bounds
      if (global_x < 0 || global_x >= static_cast<int>(global_map.info.width) ||
          global_y < 0 || global_y >= static_cast<int>(global_map.info.height)) {
        continue;
      }

      int local_idx = y * local_costmap.info.width + x;
      int global_idx = global_y * global_map.info.width + global_x;

      // Safety check for index range
      if (global_idx < 0 || global_idx >= static_cast<int>(global_map.data.size())) {
        continue;
      }

      // Copy if known cell
      int8_t cost = local_costmap.data[local_idx];
      if (cost >= 0) {
        global_map.data[global_idx] = cost;
      }
    }
  }
}

}  // namespace robot
