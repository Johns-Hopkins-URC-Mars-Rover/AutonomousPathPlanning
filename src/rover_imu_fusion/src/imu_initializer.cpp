#include "rover_imu_fusion/imu_initializer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rover_imu_fusion
{

void ImuInitializer::Stream::add(const Eigen::Vector3d & v)
{
  ++n;
  const Eigen::Vector3d delta = v - mean;
  mean += delta / static_cast<double>(n);
  m2 += delta.cwiseProduct(v - mean);
}

Eigen::Vector3d ImuInitializer::Stream::stddev() const
{
  if (n < 2) {
    return Eigen::Vector3d::Zero();
  }
  return (m2 / static_cast<double>(n - 1)).cwiseSqrt();
}

ImuInitializer::ImuInitializer(const InitConfig & config)
: cfg_(config) {}

void ImuInitializer::touch(double t)
{
  if (!window_open_) {
    window_start_ = t;
    window_open_ = true;
  }
}

void ImuInitializer::clearWindow()
{
  gyro_ = {};
  accel_ = {};
  window_open_ = false;
}

void ImuInitializer::addGyro(int imu, double t, const Eigen::Vector3d & w)
{
  if (ready_ || imu < 0 || imu > 1) {
    return;
  }
  touch(t);
  gyro_[static_cast<std::size_t>(imu)].add(w);
  tryFinish(t);
}

void ImuInitializer::addAccel(int imu, double t, const Eigen::Vector3d & f)
{
  if (ready_ || imu < 0 || imu > 1) {
    return;
  }
  touch(t);
  accel_[static_cast<std::size_t>(imu)].add(f);
  tryFinish(t);
}

void ImuInitializer::tryFinish(double t)
{
  if (!window_open_ || t - window_start_ < cfg_.duration) {
    return;
  }

  // Streams with too few samples are ignored; at least one gyro and one accel
  // stream must be usable.
  Eigen::Vector3d f_sum = Eigen::Vector3d::Zero();
  int accel_streams = 0;
  int gyro_streams = 0;
  bool still = true;
  double min_gyro_mean = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < 2; ++i) {
    if (gyro_[i].n >= cfg_.min_samples) {
      ++gyro_streams;
      min_gyro_mean = std::min(min_gyro_mean, gyro_[i].mean.norm());
      still = still && gyro_[i].stddev().maxCoeff() <= cfg_.gyro_std_max;
    }
    if (accel_[i].n >= cfg_.min_samples) {
      ++accel_streams;
      f_sum += accel_[i].mean;
      still = still && accel_[i].stddev().maxCoeff() <= cfg_.accel_std_max;
    }
  }
  if (gyro_streams == 0 || accel_streams == 0) {
    // Not enough data yet: keep collecting rather than restarting.
    return;
  }
  const Eigen::Vector3d f = f_sum / accel_streams;
  if (!still || min_gyro_mean > cfg_.gyro_mean_max ||
    std::abs(f.norm() - cfg_.gravity) > cfg_.accel_norm_tol)
  {
    ++restarts_;
    clearWindow();
    return;
  }

  NominalState s;
  s.q = Eigen::Quaterniond::FromTwoVectors(f.normalized(), Eigen::Vector3d::UnitZ());
  s.q.normalize();
  s.a = f;
  for (std::size_t i = 0; i < 2; ++i) {
    if (gyro_[i].n >= cfg_.min_samples) {
      s.bg[i] = gyro_[i].mean;
    }
  }
  state_ = s;
  time_ = t;
  ready_ = true;
}

}  // namespace rover_imu_fusion
