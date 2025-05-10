#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot {

class MapMemoryCore {
public:
  // Takes a logger for printing info if needed
  explicit MapMemoryCore(const rclcpp::Logger& logger);

  // Adds the local costmap to the global one based on robot's pose
  void fuseCostmapIntoMap(
    nav_msgs::msg::OccupancyGrid& global_map,
    const nav_msgs::msg::OccupancyGrid& local_costmap,
    const geometry_msgs::msg::Pose& robot_pose);

private:
  rclcpp::Logger logger_;
};

}  // namespace robot

#endif  // MAP_MEMORY_CORE_HPP_
