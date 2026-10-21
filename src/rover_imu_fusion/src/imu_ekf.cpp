#include "rover_imu_fusion/imu_ekf.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace rover_imu_fusion
{

namespace
{

constexpr double kSmallAngle = 1e-12;

bool validValue(double v) {return std::isfinite(v) && v >= 0.0;}

}  // namespace

Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Quaterniond expQuat(const Eigen::Vector3d & rotvec)
{
  const double angle = rotvec.norm();
  if (angle < kSmallAngle) {
    Eigen::Quaterniond q(1.0, 0.5 * rotvec.x(), 0.5 * rotvec.y(), 0.5 * rotvec.z());
    q.normalize();
    return q;
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotvec / angle));
}

Eigen::Vector3d logQuat(const Eigen::Quaterniond & q_in)
{
  // Take the shorter rotation (w >= 0).
  Eigen::Quaterniond q = q_in.normalized();
  if (q.w() < 0.0) {
    q.coeffs() = -q.coeffs();
  }
  const double s = q.vec().norm();
  if (s < kSmallAngle) {
    return 2.0 * q.vec();
  }
  return 2.0 * std::atan2(s, q.w()) * q.vec() / s;
}

void EkfConfig::validate() const
{
  const bool ok = validValue(rate_walk) && validValue(accel_walk) &&
    validValue(init_attitude_sigma) && validValue(init_rate_sigma) &&
    validValue(init_accel_sigma);
  bool per_imu_ok = true;
  for (int i = 0; i < 2; ++i) {
    const auto k = static_cast<std::size_t>(i);
    per_imu_ok = per_imu_ok && validValue(gyro_bias_walk[k]) && validValue(accel_bias_walk[k]) &&
      validValue(init_gyro_bias_sigma[k]) && validValue(init_accel_bias_sigma[k]);
  }
  if (!ok || !per_imu_ok) {
    throw std::invalid_argument("EkfConfig: every noise value must be set, finite and >= 0");
  }
  if (!validValue(gravity_sigma) || !(gravity > 0.0) || !validValue(gravity_adapt) ||
    gravity_adapt <= 0.0 || !validValue(gravity_skip))
  {
    throw std::invalid_argument("EkfConfig: gravity parameters invalid or gravity_sigma unset");
  }
  if (!(nis_gate_3dof > 0.0) || !(nis_gate_1dof > 0.0) || stuck_count < 0 ||
    !validValue(gyro_saturation[0]) || !validValue(gyro_saturation[1]) ||
    !validValue(accel_saturation[0]) || !validValue(accel_saturation[1]))
  {
    throw std::invalid_argument("EkfConfig: gate, saturation or stuck_count invalid");
  }
}

ImuEkf::ImuEkf(const EkfConfig & config)
: cfg_(config)
{
  cfg_.validate();
}

void ImuEkf::initialize(const NominalState & state, double time)
{
  x_ = state;
  x_.q.normalize();
  time_ = time;

  P_.setZero();
  P_.block<3, 3>(idx::kTheta, idx::kTheta).diagonal().setConstant(
    cfg_.init_attitude_sigma * cfg_.init_attitude_sigma);
  P_.block<3, 3>(idx::kRate, idx::kRate).diagonal().setConstant(
    cfg_.init_rate_sigma * cfg_.init_rate_sigma);
  P_.block<3, 3>(idx::kAccel, idx::kAccel).diagonal().setConstant(
    cfg_.init_accel_sigma * cfg_.init_accel_sigma);
  for (int i = 0; i < 2; ++i) {
    const auto k = static_cast<std::size_t>(i);
    P_.block<3, 3>(idx::gyroBias(i), idx::gyroBias(i)).diagonal().setConstant(
      cfg_.init_gyro_bias_sigma[k] * cfg_.init_gyro_bias_sigma[k]);
    P_.block<3, 3>(idx::accelBias(i), idx::accelBias(i)).diagonal().setConstant(
      cfg_.init_accel_bias_sigma[k] * cfg_.init_accel_bias_sigma[k]);
  }
  counters_ = {};
  gyro_health_ = {};
  accel_health_ = {};
  gyro_consec_nis_ = {};
  accel_consec_nis_ = {};
  gravity_applied_ = 0;
  gravity_skipped_ = 0;
  has_mag_ref_ = false;
  initialized_ = true;
}

StateCov ImuEkf::transition(double dt) const
{
  StateCov F = StateCov::Identity();
  const Eigen::Vector3d phi = x_.w * dt;
  F.block<3, 3>(idx::kTheta, idx::kTheta) = expQuat(-phi).toRotationMatrix();
  F.block<3, 3>(idx::kTheta, idx::kRate) =
    dt * (Eigen::Matrix3d::Identity() - 0.5 * skew(phi));
  return F;
}

bool ImuEkf::predict(double t)
{
  if (!initialized_) {
    throw std::logic_error("ImuEkf::predict called before initialize");
  }
  const double dt = t - time_;
  if (dt < 0.0) {
    return false;
  }
  if (dt == 0.0) {
    return true;
  }

  const StateCov F = transition(dt);

  // Random-walk process noise, diagonal.
  StateCov Q = StateCov::Zero();
  Q.block<3, 3>(
    idx::kRate,
    idx::kRate).diagonal().setConstant(cfg_.rate_walk * cfg_.rate_walk * dt);
  Q.block<3, 3>(idx::kAccel, idx::kAccel).diagonal().setConstant(
    cfg_.accel_walk * cfg_.accel_walk * dt);
  for (int i = 0; i < 2; ++i) {
    const auto k = static_cast<std::size_t>(i);
    Q.block<3, 3>(idx::gyroBias(i), idx::gyroBias(i)).diagonal().setConstant(
      cfg_.gyro_bias_walk[k] * cfg_.gyro_bias_walk[k] * dt);
    Q.block<3, 3>(idx::accelBias(i), idx::accelBias(i)).diagonal().setConstant(
      cfg_.accel_bias_walk[k] * cfg_.accel_bias_walk[k] * dt);
  }

  P_ = F * P_ * F.transpose() + Q;
  P_ = 0.5 * (P_ + P_.transpose());

  x_.q = x_.q * expQuat(x_.w * dt);
  x_.q.normalize();
  time_ = t;
  return true;
}

const ImuCounters & ImuEkf::counters(int imu) const
{
  if (imu < 0 || imu > 1) {
    throw std::out_of_range("imu index must be 0 or 1");
  }
  return counters_[static_cast<std::size_t>(imu)];
}

UpdateStatus ImuEkf::checkHealth(
  StreamHealth & h, const Eigen::Vector3d & z, double limit) const
{
  if (limit > 0.0 && z.norm() >= limit) {
    return UpdateStatus::kRejectedSaturated;
  }
  if (cfg_.stuck_count > 0) {
    if (h.has_last && z == h.last) {
      ++h.repeats;
    } else {
      h.repeats = 0;
    }
    h.last = z;
    h.has_last = true;
    if (h.repeats >= cfg_.stuck_count) {
      return UpdateStatus::kRejectedStuck;
    }
  }
  return UpdateStatus::kApplied;
}

void ImuEkf::inject(const StateVec & dx)
{
  const Eigen::Vector3d dtheta = dx.segment<3>(idx::kTheta);
  x_.q = x_.q * expQuat(dtheta);
  x_.q.normalize();
  x_.w += dx.segment<3>(idx::kRate);
  x_.a += dx.segment<3>(idx::kAccel);
  for (int i = 0; i < 2; ++i) {
    const auto k = static_cast<std::size_t>(i);
    x_.bg[k] += dx.segment<3>(idx::gyroBias(i));
    x_.ba[k] += dx.segment<3>(idx::accelBias(i));
  }

  // Covariance reset: the attitude error is now expressed about the new q.
  StateCov G = StateCov::Identity();
  G.block<3, 3>(idx::kTheta, idx::kTheta) = Eigen::Matrix3d::Identity() - 0.5 * skew(dtheta);
  P_ = G * P_ * G.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
}

template<int M>
UpdateStatus ImuEkf::applyUpdate(
  const Eigen::Matrix<double, M, kStateDim> & H, const Eigen::Matrix<double, M, 1> & y,
  const Eigen::Matrix<double, M, M> & R, double gate)
{
  const Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, M, M> S_inv = S.inverse();
  const double nis = (y.transpose() * S_inv * y).value();
  last_nis_ = nis;
  if (!(nis <= gate)) {  // also rejects NaN
    return UpdateStatus::kRejectedNis;
  }
  const Eigen::Matrix<double, kStateDim, M> K = P_ * H.transpose() * S_inv;
  const StateVec dx = K * y;
  const StateCov I_KH = StateCov::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();  // Joseph form
  P_ = 0.5 * (P_ + P_.transpose());
  inject(dx);
  return UpdateStatus::kApplied;
}

UpdateStatus ImuEkf::vectorUpdate(
  int imu, double t, const Eigen::Vector3d & z, const Eigen::Matrix3d & R, bool gyro)
{
  if (!initialized_) {
    throw std::logic_error("ImuEkf update called before initialize");
  }
  if (imu < 0 || imu > 1) {
    throw std::out_of_range("imu index must be 0 or 1");
  }
  const auto k = static_cast<std::size_t>(imu);
  ImuCounters & c = counters_[k];

  if (!predict(t)) {
    ++c.stale;
    return UpdateStatus::kStale;
  }
  c.last_time = t;

  const double limit = gyro ? cfg_.gyro_saturation[k] : cfg_.accel_saturation[k];
  const UpdateStatus health = checkHealth(gyro ? gyro_health_[k] : accel_health_[k], z, limit);
  if (health == UpdateStatus::kRejectedSaturated) {
    ++c.rejected_saturated;
    return health;
  }
  if (health == UpdateStatus::kRejectedStuck) {
    ++c.rejected_stuck;
    return health;
  }

  Eigen::Matrix<double, 3, kStateDim> H = Eigen::Matrix<double, 3, kStateDim>::Zero();
  Eigen::Vector3d predicted;
  if (gyro) {
    H.block<3, 3>(0, idx::kRate).setIdentity();
    H.block<3, 3>(0, idx::gyroBias(imu)).setIdentity();
    predicted = x_.w + x_.bg[k];
  } else {
    H.block<3, 3>(0, idx::kAccel).setIdentity();
    H.block<3, 3>(0, idx::accelBias(imu)).setIdentity();
    predicted = x_.a + x_.ba[k];
  }
  int & consec = gyro ? gyro_consec_nis_[k] : accel_consec_nis_[k];
  const bool force = cfg_.nis_max_consecutive > 0 && consec >= cfg_.nis_max_consecutive;
  const double gate = force ? std::numeric_limits<double>::infinity() : cfg_.nis_gate_3dof;
  const UpdateStatus status = applyUpdate<3>(H, z - predicted, R, gate);
  if (status == UpdateStatus::kApplied) {
    ++c.applied;
    if (force) {
      ++c.forced;
    }
    consec = 0;
  } else {
    ++c.rejected_nis;
    ++consec;
  }
  return status;
}

UpdateStatus ImuEkf::updateGyro(
  int imu, double t, const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
{
  return vectorUpdate(imu, t, z, R, true);
}

UpdateStatus ImuEkf::updateAccel(
  int imu, double t, const Eigen::Vector3d & z, const Eigen::Matrix3d & R)
{
  return vectorUpdate(imu, t, z, R, false);
}

UpdateStatus ImuEkf::updateGravity(double t)
{
  if (!initialized_) {
    throw std::logic_error("ImuEkf update called before initialize");
  }
  if (!predict(t)) {
    return UpdateStatus::kStale;
  }
  const double dyn = x_.a.norm() - cfg_.gravity;
  if (std::abs(dyn) > cfg_.gravity_skip) {
    ++gravity_skipped_;
    return UpdateStatus::kSkipped;
  }
  const double ratio = dyn / cfg_.gravity_adapt;
  const double var = cfg_.gravity_sigma * cfg_.gravity_sigma * (1.0 + ratio * ratio);

  const Eigen::Vector3d g_body =
    x_.q.toRotationMatrix().transpose() * Eigen::Vector3d(0.0, 0.0, cfg_.gravity);
  Eigen::Matrix<double, 3, kStateDim> H = Eigen::Matrix<double, 3, kStateDim>::Zero();
  H.block<3, 3>(0, idx::kTheta) = -skew(g_body);
  H.block<3, 3>(0, idx::kAccel).setIdentity();

  const UpdateStatus status = applyUpdate<3>(
    H, g_body - x_.a, var * Eigen::Matrix3d::Identity(), cfg_.nis_gate_3dof);
  if (status == UpdateStatus::kApplied) {
    ++gravity_applied_;
  } else {
    ++gravity_skipped_;
  }
  return status;
}

UpdateStatus ImuEkf::updateMagYaw(double t, const Eigen::Vector3d & m_body)
{
  if (!initialized_) {
    throw std::logic_error("ImuEkf update called before initialize");
  }
  if (!validValue(cfg_.mag_yaw_sigma) || cfg_.mag_yaw_sigma <= 0.0) {
    throw std::logic_error("mag_yaw_sigma must be set to use the magnetometer");
  }
  if (!predict(t)) {
    return UpdateStatus::kStale;
  }
  const Eigen::Matrix3d R = x_.q.toRotationMatrix();
  const Eigen::Vector3d m_w = R * m_body;
  if (!m_w.allFinite() || m_w.head<2>().norm() < 1e-12) {
    return UpdateStatus::kSkipped;
  }
  const double heading = std::atan2(m_w.y(), m_w.x());
  if (!has_mag_ref_) {
    mag_ref_heading_ = heading;
    has_mag_ref_ = true;
    return UpdateStatus::kSkipped;
  }
  double err = mag_ref_heading_ - heading;
  err = std::atan2(std::sin(err), std::cos(err));

  Eigen::Matrix<double, 1, kStateDim> H = Eigen::Matrix<double, 1, kStateDim>::Zero();
  H.block<1, 3>(0, idx::kTheta) = (R.transpose() * Eigen::Vector3d::UnitZ()).transpose();
  Eigen::Matrix<double, 1, 1> y;
  y(0, 0) = err;
  Eigen::Matrix<double, 1, 1> Rm;
  Rm(0, 0) = cfg_.mag_yaw_sigma * cfg_.mag_yaw_sigma;
  return applyUpdate<1>(H, y, Rm, cfg_.nis_gate_1dof);
}

Eigen::Matrix3d ImuEkf::orientationCovarianceWorld() const
{
  const Eigen::Matrix3d R = x_.q.toRotationMatrix();
  const Eigen::Matrix3d P_theta = P_.block<3, 3>(idx::kTheta, idx::kTheta);
  return R * P_theta * R.transpose();
}

}  // namespace rover_imu_fusion
