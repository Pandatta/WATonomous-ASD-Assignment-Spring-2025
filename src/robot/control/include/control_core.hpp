#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot {

class ControlCore {
public:
  ControlCore(double lookahead, double tolerance, double speed);

  geometry_msgs::msg::PoseStamped findTarget(const nav_msgs::msg::Path& path,
                                              const geometry_msgs::msg::Pose& current_pose);

  geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::Pose& current_pose,
                                            const geometry_msgs::msg::PoseStamped& target_pose);

  bool goalReached(const geometry_msgs::msg::Pose& current_pose,
                   const geometry_msgs::msg::Pose& goal_pose);

private:
  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;

  double getYaw(const geometry_msgs::msg::Quaternion& q);
  double normalizeAngle(double angle);
  double distance(double x1, double y1, double x2, double y2);
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
