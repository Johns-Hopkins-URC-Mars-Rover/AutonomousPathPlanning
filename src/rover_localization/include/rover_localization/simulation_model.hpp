#ifndef ROVER_LOCALIZATION__SIMULATION_MODEL_HPP_
#define ROVER_LOCALIZATION__SIMULATION_MODEL_HPP_

#include <cmath>

namespace rover_localization {

struct PlanarPose {
  double x;
  double y;
  double yaw;
};

inline double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline PlanarPose advance_pose(
    const PlanarPose &pose, double linear_speed, double angular_speed, double dt) {
  PlanarPose next = pose;
  const double next_yaw = pose.yaw + angular_speed * dt;
  if (std::abs(angular_speed) < 1e-6) {
    next.x += linear_speed * std::cos(pose.yaw) * dt;
    next.y += linear_speed * std::sin(pose.yaw) * dt;
  } else {
    const double radius = linear_speed / angular_speed;
    next.x += radius * (std::sin(next_yaw) - std::sin(pose.yaw));
    next.y += radius * (std::cos(pose.yaw) - std::cos(next_yaw));
  }
  next.yaw = wrap_angle(next_yaw);
  return next;
}

}  // namespace rover_localization

#endif  // ROVER_LOCALIZATION__SIMULATION_MODEL_HPP_
