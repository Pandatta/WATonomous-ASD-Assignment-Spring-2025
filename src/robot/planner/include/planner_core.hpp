#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"

#include <unordered_map>
#include <queue>

// Simple structure to hold grid coordinates
struct GridIndex {
  int x;
  int y;

  GridIndex(int x_val, int y_val) : x(x_val), y(y_val) {}
  GridIndex() : x(0), y(0) {}

  bool operator==(const GridIndex& other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const GridIndex& other) const {
    return !(*this == other);
  }
};

// Helper to allow GridIndex to be used in unordered_map
struct GridIndexHash {
  std::size_t operator()(const GridIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// A* node used for the priority queue
struct AStarNode {
  GridIndex index;
  double f_score;

  AStarNode(GridIndex idx, double f) : index(idx), f_score(f) {}
};

// Compare nodes based on f-score (lower is better)
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    return a.f_score > b.f_score;
  }
};

// Main path planner logic
class PlannerCore {
public:
  static nav_msgs::msg::Path aStarPlan(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Point& goal);
};

#endif  // PLANNER_CORE_HPP_
