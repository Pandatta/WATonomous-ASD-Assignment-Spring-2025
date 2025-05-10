#include "planner_core.hpp"
#include <cmath>
#include <algorithm>

nav_msgs::msg::Path PlannerCore::aStarPlan(
  const nav_msgs::msg::OccupancyGrid& map,
  const geometry_msgs::msg::Pose& start,
  const geometry_msgs::msg::Point& goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";

  int width = map.info.width;
  int height = map.info.height;
  double res = map.info.resolution;
  double origin_x = map.info.origin.position.x;
  double origin_y = map.info.origin.position.y;

  auto toIndex = [&](double x, double y) {
    return GridIndex(
      static_cast<int>((x - origin_x) / res),
      static_cast<int>((y - origin_y) / res)
    );
  };

  auto toPose = [&](const GridIndex& idx) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = idx.x * res + origin_x + res / 2.0;
    pose.pose.position.y = idx.y * res + origin_y + res / 2.0;
    pose.pose.orientation.w = 1.0;
    return pose;
  };

  auto isFree = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= width || y >= height) return false;
    int8_t val = map.data[y * width + x];
    return val >= 0 && val < 100;
  };

  auto heuristic = [&](const GridIndex& a, const GridIndex& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  GridIndex start_idx = toIndex(start.position.x, start.position.y);
  GridIndex goal_idx = toIndex(goal.x, goal.y);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_map<GridIndex, bool, GridIndexHash> visited;

  g_score[start_idx] = 0.0;
  open_set.emplace(start_idx, heuristic(start_idx, goal_idx));

  std::vector<std::pair<int, int>> directions = {
    {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {-1,-1}, {1,-1}, {-1,1}
  };

  while (!open_set.empty()) {
    auto current = open_set.top();
    open_set.pop();

    if (current.index == goal_idx) {
      GridIndex idx = goal_idx;
      while (idx != start_idx) {
        path.poses.push_back(toPose(idx));
        idx = came_from[idx];
      }
      path.poses.push_back(toPose(start_idx));
      std::reverse(path.poses.begin(), path.poses.end());
      return path;
    }

    visited[current.index] = true;

    for (auto [dx, dy] : directions) {
      GridIndex neighbor(current.index.x + dx, current.index.y + dy);
      if (!isFree(neighbor.x, neighbor.y) || visited[neighbor]) continue;

      double tentative_g = g_score[current.index] + heuristic(current.index, neighbor);
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current.index;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal_idx);
        open_set.emplace(neighbor, f);
      }
    }
  }

  return path; // return empty path if no route is found
}
