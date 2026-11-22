#pragma once

#include <Eigen/Dense>

#include <array>
#include <cstddef>

#include "rover_imu_fusion/imu_ekf.hpp"

namespace rover_imu_fusion
{

// Detection thresholds for the stationary start. These are not sensor noise
// values; they decide whether the rover is still enough to seed the filter.
struct InitConfig
{
  double duration{1.0};         // s of stationary data to average
  std::size_t min_samples{20};  // per stream, otherwise the stream is ignored
  double gyro_std_max{0.05};    // rad/s, max per-axis std of a stream
  double accel_std_max{0.5};    // m/s^2, max per-axis std of a stream
  // At least one gyro stream must have a mean rate norm below this (rad/s). A
  // rotating rover cannot make a bias-corrected IMU read near zero, while a
  // real gyro bias (a few dps) stays below it.
  double gyro_mean_max{0.1};
  double gravity{9.80665};      // m/s^2
  double accel_norm_tol{1.0};   // m/s^2, allowed | |mean a| - g |
};

// Averages gyro and accel samples (base_link axes) over a stationary window and
// builds the initial nominal state:
//   roll/pitch : from the mean specific force, q0 = FromTwoVectors(f, z_world),
//                yaw 0 (relative)
//   w          : 0
//   a          : mean specific force over the IMUs that reported
//   bg_i       : mean gyro of IMU i (the rover is still, so w = 0)
//   ba_i       : 0 (not separable from gravity at rest)
// If the window is not stationary it restarts.
class ImuInitializer
{
public:
  explicit ImuInitializer(const InitConfig & config = InitConfig{});

  void addGyro(int imu, double t, const Eigen::Vector3d & w);
  void addAccel(int imu, double t, const Eigen::Vector3d & f);

  bool ready() const {return ready_;}
  const NominalState & state() const {return state_;}
  double time() const {return time_;}
  std::size_t restarts() const {return restarts_;}

private:
  struct Stream
  {
    std::size_t n{0};
    Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
    Eigen::Vector3d m2{Eigen::Vector3d::Zero()};  // Welford sum of squared deviations

    void add(const Eigen::Vector3d & v);
    Eigen::Vector3d stddev() const;
  };

  InitConfig cfg_;
  std::array<Stream, 2> gyro_;
  std::array<Stream, 2> accel_;
  double window_start_{0.0};
  bool window_open_{false};
  bool ready_{false};
  NominalState state_;
  double time_{0.0};
  std::size_t restarts_{0};

  void touch(double t);
  void clearWindow();
  void tryFinish(double t);
};

}  // namespace rover_imu_fusion
