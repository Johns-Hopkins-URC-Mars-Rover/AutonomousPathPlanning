#include "simulation_model.hpp"

#include <stdexcept>

static void require(bool ok) {
  if (!ok) throw std::runtime_error("motion-model check failed");
}
#include <cmath>

int main() {
  using rover_localization::PlanarPose;
  using rover_localization::advance_pose;
  using rover_localization::wrap_angle;

  const PlanarPose start{0.0, 0.0, 0.0};
  const auto straight = advance_pose(start, 1.0, 0.0, 2.0);
  require(std::abs(straight.x - 2.0) < 1e-9);
  require(std::abs(straight.y) < 1e-9);
  require(std::abs(straight.yaw) < 1e-9);

  const auto turn = advance_pose(start, 1.0, 1.0, 1.0);
  require(std::abs(turn.x - std::sin(1.0)) < 1e-9);
  require(std::abs(turn.y - (1.0 - std::cos(1.0))) < 1e-9);
  require(std::abs(turn.yaw - 1.0) < 1e-9);

  require(std::abs(wrap_angle(4.0 * std::acos(-1.0))) < 1e-9);
}
