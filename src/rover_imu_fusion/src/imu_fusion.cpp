#include "rover_imu_fusion/imu_fusion.hpp"

#include <algorithm>

namespace rover_imu_fusion
{

ImuFusion::ImuFusion(const FusionConfig & config)
: cfg_(config), buffer_(config.buffer_size), init_(config.init)
{
  cfg_.ekf.validate();
}

void ImuFusion::add(const Measurement & m)
{
  // Drop samples with an invalid IMU index; the magnetometer has no IMU index.
  if (m.kind != Measurement::Kind::kMag && (m.imu < 0 || m.imu > 1)) {
    return;
  }
  // Track the newest stamp per IMU for the timeout check. This is done on
  // arrival, before reordering, so a live IMU is seen as live immediately.
  if (m.kind != Measurement::Kind::kMag) {
    auto & last = last_stamp_[static_cast<std::size_t>(m.imu)];
    last = std::max(last, m.stamp);
  }
  buffer_.push(m.stamp, m);
}

void ImuFusion::process(double cutoff)
{
  // Release everything old enough to be in order and feed it to the filter.
  for (const Measurement & m : buffer_.popUntil(cutoff)) {
    handle(m);
  }
  // Keep the output current even when no measurement arrived.
  if (ekf_) {
    ekf_->predict(cutoff);  // a stale cutoff is ignored
  }
}

void ImuFusion::handle(const Measurement & m)
{
  // Before the filter exists, samples only feed the stationary initializer
  // (the magnetometer is ignored). The measurement that completes the window
  // seeds the filter and is itself not applied as an update.
  if (!ekf_) {
    if (m.kind == Measurement::Kind::kGyro) {
      init_.addGyro(m.imu, m.stamp, m.z);
    } else if (m.kind == Measurement::Kind::kAccel) {
      init_.addAccel(m.imu, m.stamp, m.z);
    }
    if (init_.ready()) {
      ekf_.emplace(cfg_.ekf);
      ekf_->initialize(init_.state(), init_.time());
    }
    return;
  }

  switch (m.kind) {
    case Measurement::Kind::kGyro:
      ekf_->updateGyro(m.imu, m.stamp, m.z, m.R);
      break;
    case Measurement::Kind::kAccel:
      ekf_->updateAccel(m.imu, m.stamp, m.z, m.R);
      // The gravity pseudo-measurement is rate-limited and piggybacks on accel
      // samples, so it uses the freshly updated acceleration estimate.
      if (m.stamp - last_gravity_ >= cfg_.gravity_update_period) {
        // Roll/pitch aid; it is only applied when the filter time is current.
        if (ekf_->updateGravity(m.stamp) != UpdateStatus::kStale) {
          last_gravity_ = m.stamp;
        }
      }
      break;
    case Measurement::Kind::kMag:
      ekf_->updateMagYaw(m.stamp, m.z);
      break;
  }
}

std::optional<FusionOutput> ImuFusion::output() const
{
  if (!ekf_) {
    return std::nullopt;
  }
  FusionOutput out;
  out.stamp = ekf_->time();
  out.q = ekf_->state().q;
  // Orientation covariance about world axes; w and a covariances are the
  // matching diagonal blocks of P (already in base_link axes).
  out.q_cov = ekf_->orientationCovarianceWorld();
  out.w = ekf_->state().w;
  out.w_cov = ekf_->covariance().block<3, 3>(idx::kRate, idx::kRate);
  out.a = ekf_->state().a;
  out.a_cov = ekf_->covariance().block<3, 3>(idx::kAccel, idx::kAccel);
  return out;
}

bool ImuFusion::timedOut(int imu, double now) const
{
  if (imu < 0 || imu > 1) {
    return true;
  }
  const double last = last_stamp_[static_cast<std::size_t>(imu)];
  // Never seen (-1) counts as timed out.
  return last < 0.0 || now - last > cfg_.imu_timeout;
}

}  // namespace rover_imu_fusion
