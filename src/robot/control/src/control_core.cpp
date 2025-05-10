#include "control_core.hpp"
#include <cmath>

namespace robot {

ControlCore::ControlCore(double lookahead, double tolerance, double speed)
  : lookahead_distance_(lookahead),
    goal_tolerance_(tolerance),
    linear_speed_(speed) {}

double ControlCore::distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

double ControlCore::getYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double ControlCore::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

geometry_msgs::msg::PoseStamped ControlCore::findTarget(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Pose& current_pose) {
  for (const auto& pose_stamped : path.poses) {
    double dist = distance(current_pose.position.x, current_pose.position.y,
                           pose_stamped.pose.position.x, pose_stamped.pose.position.y);
    if (dist > lookahead_distance_) {
      return pose_stamped;
    }
  }
  return path.poses.back();  // fallback to last
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::Pose& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose) {
  geometry_msgs::msg::Twist cmd;

  double dx = target_pose.pose.position.x - current_pose.position.x;
  double dy = target_pose.pose.position.y - current_pose.position.y;
  double target_angle = std::atan2(dy, dx);
  double current_yaw = getYaw(current_pose.orientation);

  double error = normalizeAngle(target_angle - current_yaw);

  cmd.linear.x = linear_speed_;
  cmd.angular.z = 1.5 * error;
  return cmd;
}

bool ControlCore::goalReached(const geometry_msgs::msg::Pose& current_pose,
                              const geometry_msgs::msg::Pose& goal_pose) {
  return distance(current_pose.position.x, current_pose.position.y,
                  goal_pose.position.x, goal_pose.position.y) < goal_tolerance_;
}

}  // namespace robot
