// Standalone regression test: no ROS/gtest required; checks remain on in Release.
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "rover_imu_fusion/imu_fusion.hpp"

using namespace rover_imu_fusion;
static void require(bool ok, const char * message) {
  if (!ok) throw std::runtime_error(message);
}
static FusionConfig config() {
  FusionConfig c;
  c.ekf.rate_walk = .02; c.ekf.accel_walk = .3;
  c.ekf.gyro_bias_walk = {1e-5, 1e-5}; c.ekf.accel_bias_walk = {1e-4, 1e-4};
  c.ekf.init_attitude_sigma = .01; c.ekf.init_rate_sigma = .01; c.ekf.init_accel_sigma = .1;
  c.ekf.init_gyro_bias_sigma = {.01, .002}; c.ekf.init_accel_bias_sigma = {.05, .05};
  c.ekf.gravity_sigma = .3;
  return c;
}
int main() {
  ImuFusion f(config());
  Measurement m; m.imu = 1; m.stamp = 1.; m.R.setIdentity();
  m.z.x() = std::numeric_limits<double>::quiet_NaN();
  f.add(m);
  require(f.lastStamp(1) < 0., "nonfinite measurement must not enter buffer or count as live");
  m.z.setZero(); m.R(0,0) = -1.; f.add(m);
  require(f.lastStamp(1) < 0., "invalid covariance must not count as live");
  for (int i = 0; i < 150; ++i) {
    m.stamp = 2. + .01*i; m.kind = Measurement::Kind::kGyro; m.z.setZero();
    m.R = .001 * Eigen::Matrix3d::Identity(); f.add(m);
    m.kind = Measurement::Kind::kAccel; m.z = Eigen::Vector3d(0,0,9.80665); f.add(m);
    f.process(m.stamp);
  }
  require(f.output().has_value(), "valid stationary input must initialize and publish");
  f.process(10.);
  require(!f.output().has_value(), "all IMUs timed out: do not publish fresh-looking predictions");
  std::cout << "PASS: finite input, covariance, initialization, complete dropout\n";
}
