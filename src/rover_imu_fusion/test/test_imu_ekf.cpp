#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <stdexcept>

#include "rover_imu_fusion/imu_ekf.hpp"

using rover_imu_fusion::EkfConfig;
using rover_imu_fusion::expQuat;
using rover_imu_fusion::ImuEkf;
using rover_imu_fusion::logQuat;
using rover_imu_fusion::NominalState;
using rover_imu_fusion::StateCov;
using rover_imu_fusion::UpdateStatus;
namespace idx = rover_imu_fusion::idx;

namespace
{

// Numbers are arbitrary test values, not sensor properties.
EkfConfig testConfig()
{
  EkfConfig c;
  c.rate_walk = 0.05;
  c.accel_walk = 0.5;
  c.gyro_bias_walk = {1e-4, 1e-4};
  c.accel_bias_walk = {1e-3, 1e-3};
  c.init_attitude_sigma = 0.01;
  c.init_rate_sigma = 0.02;
  c.init_accel_sigma = 0.1;
  c.init_gyro_bias_sigma = {0.01, 0.01};
  c.init_accel_bias_sigma = {0.05, 0.05};
  c.gravity_sigma = 0.3;
  c.mag_yaw_sigma = 0.05;
  return c;
}

}  // namespace

TEST(ExpLog, RoundTrip)
{
  const Eigen::Vector3d v(0.3, -0.2, 0.5);
  EXPECT_TRUE(logQuat(expQuat(v)).isApprox(v, 1e-12));
  EXPECT_TRUE(logQuat(expQuat(Eigen::Vector3d::Zero())).isZero(1e-15));
}

TEST(EkfConfig, ThrowsWhenNoiseNotSet)
{
  EkfConfig c;  // all unset
  EXPECT_THROW(ImuEkf{c}, std::invalid_argument);
  c = testConfig();
  c.accel_walk = -1.0;
  EXPECT_THROW(ImuEkf{c}, std::invalid_argument);
}

TEST(Predict, ThrowsBeforeInitialize)
{
  ImuEkf ekf(testConfig());
  EXPECT_THROW(ekf.predict(1.0), std::logic_error);
}

TEST(Predict, StaleTimeIsRejectedAndChangesNothing)
{
  ImuEkf ekf(testConfig());
  ekf.initialize(NominalState{}, 10.0);
  const StateCov before = ekf.covariance();
  EXPECT_FALSE(ekf.predict(9.99));
  EXPECT_DOUBLE_EQ(ekf.time(), 10.0);
  EXPECT_EQ(ekf.covariance(), before);
}

TEST(Predict, StationaryKeepsAttitudeAndYawVarianceGrowsMonotonically)
{
  ImuEkf ekf(testConfig());
  NominalState s;
  s.a = Eigen::Vector3d(0, 0, 9.81);
  ekf.initialize(s, 0.0);

  double prev = ekf.orientationCovarianceWorld()(2, 2);
  for (int k = 1; k <= 2000; ++k) {
    ASSERT_TRUE(ekf.predict(0.01 * k));
    const double yaw_var = ekf.orientationCovarianceWorld()(2, 2);
    ASSERT_GT(yaw_var, prev) << "step " << k;
    prev = yaw_var;
  }
  EXPECT_TRUE(ekf.state().q.isApprox(Eigen::Quaterniond::Identity(), 1e-12));
  EXPECT_TRUE(ekf.state().w.isZero(0.0));
}

TEST(Predict, ConstantRotationIntegratesAboutEachAxis)
{
  const double rate = 0.7;  // rad/s
  const double dt = 0.005;
  const int steps = 400;    // 2 s -> 1.4 rad
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d unit = Eigen::Vector3d::Zero();
    unit(axis) = 1.0;
    NominalState s;
    s.w = rate * unit;
    ImuEkf ekf(testConfig());
    ekf.initialize(s, 0.0);
    for (int k = 1; k <= steps; ++k) {
      ASSERT_TRUE(ekf.predict(dt * k));
    }
    const Eigen::Quaterniond expected(Eigen::AngleAxisd(rate * dt * steps, unit));
    EXPECT_LT(logQuat(ekf.state().q.inverse() * expected).norm(), 1e-9) << "axis " << axis;
  }
}

TEST(Predict, CovarianceStaysSymmetricAndPositiveSemiDefinite)
{
  NominalState s;
  s.w = Eigen::Vector3d(0.3, -0.2, 0.5);
  ImuEkf ekf(testConfig());
  ekf.initialize(s, 0.0);
  for (int k = 1; k <= 1000; ++k) {
    ekf.predict(0.01 * k);
  }
  const StateCov & P = ekf.covariance();
  EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-15);
  const Eigen::SelfAdjointEigenSolver<StateCov> eig(P, Eigen::EigenvaluesOnly);
  EXPECT_GE(eig.eigenvalues().minCoeff(), -1e-12);
}

TEST(Predict, BiasAndRateCovarianceGrowWithProcessNoise)
{
  ImuEkf ekf(testConfig());
  ekf.initialize(NominalState{}, 0.0);
  const double bg_before = ekf.covariance()(idx::gyroBias(0), idx::gyroBias(0));
  const double w_before = ekf.covariance()(idx::kRate, idx::kRate);
  ekf.predict(1.0);
  EXPECT_NEAR(
    ekf.covariance()(idx::gyroBias(0), idx::gyroBias(0)) - bg_before, 1e-4 * 1e-4 * 1.0, 1e-15);
  EXPECT_NEAR(ekf.covariance()(idx::kRate, idx::kRate) - w_before, 0.05 * 0.05 * 1.0, 1e-12);
}

// The attitude part of F must match what the nominal propagation does to a
// small perturbation of the attitude and of the rate.
TEST(Transition, AttitudeBlocksMatchFiniteDifference)
{
  const double dt = 0.01;
  NominalState s;
  s.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d(1, 2, -1).normalized()));
  s.w = Eigen::Vector3d(0.8, -0.5, 1.1);
  ImuEkf ekf(testConfig());
  ekf.initialize(s, 0.0);
  const StateCov F = ekf.transition(dt);

  const Eigen::Vector3d dtheta(2e-4, -1e-4, 3e-4);
  const Eigen::Vector3d dw(-1e-4, 2e-4, 1e-4);

  // Nominal and perturbed ("true") propagation.
  const Eigen::Quaterniond q1 = s.q * expQuat(s.w * dt);
  const Eigen::Quaterniond q_true = s.q * expQuat(dtheta);
  const Eigen::Quaterniond q1_true = q_true * expQuat((s.w + dw) * dt);
  const Eigen::Vector3d actual = logQuat(q1.inverse() * q1_true);

  const Eigen::Vector3d predicted =
    F.block<3, 3>(idx::kTheta, idx::kTheta) * dtheta +
    F.block<3, 3>(idx::kTheta, idx::kRate) * dw;
  EXPECT_LT((actual - predicted).norm(), 1e-8);
}

// ---------------------------------------------------------------------------
// Measurement updates
// ---------------------------------------------------------------------------

namespace
{

Eigen::Matrix3d isoCov(double sigma)
{
  return sigma * sigma * Eigen::Matrix3d::Identity();
}

Eigen::Vector3d gravityBody(const Eigen::Quaterniond & q, double g = 9.80665)
{
  return q.toRotationMatrix().transpose() * Eigen::Vector3d(0, 0, g);
}

}  // namespace

TEST(Update, StaleMeasurementIsRejectedAndCounted)
{
  ImuEkf ekf(testConfig());
  ekf.initialize(NominalState{}, 5.0);
  EXPECT_EQ(ekf.updateGyro(0, 4.0, Eigen::Vector3d::Zero(), isoCov(0.01)), UpdateStatus::kStale);
  EXPECT_EQ(ekf.counters(0).stale, 1u);
  EXPECT_EQ(ekf.counters(0).applied, 0u);
}

TEST(Update, GyroUpdateMovesRateTowardMeasurementAndShrinksCovariance)
{
  ImuEkf ekf(testConfig());
  ekf.initialize(NominalState{}, 0.0);
  const double before = ekf.covariance()(idx::kRate, idx::kRate);
  const Eigen::Vector3d z(0.03, 0.0, 0.0);
  ASSERT_EQ(ekf.updateGyro(1, 0.01, z, isoCov(0.01)), UpdateStatus::kApplied);
  EXPECT_GT(ekf.state().w.x(), 0.005);
  EXPECT_LT(ekf.state().w.x(), 0.03);
  EXPECT_LT(ekf.covariance()(idx::kRate, idx::kRate), before);
}

TEST(Update, JosephFormKeepsCovarianceSymmetricPsd)
{
  ImuEkf ekf(testConfig());
  ekf.initialize(NominalState{}, 0.0);
  for (int k = 1; k <= 500; ++k) {
    ekf.updateGyro(k % 2, 0.01 * k, Eigen::Vector3d(0.1, 0.0, 0.0), isoCov(0.01));
    ekf.updateAccel(k % 2, 0.01 * k, Eigen::Vector3d(0, 0, 9.8), isoCov(0.05));
  }
  const StateCov & P = ekf.covariance();
  EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-15);
  const Eigen::SelfAdjointEigenSolver<StateCov> eig(P, Eigen::EigenvaluesOnly);
  EXPECT_GE(eig.eigenvalues().minCoeff(), -1e-12);
}

TEST(Update, NisGateRejectsOutlierAndLeavesStateUntouched)
{
  ImuEkf ekf(testConfig());
  ekf.initialize(NominalState{}, 0.0);
  const Eigen::Vector3d w_before = ekf.state().w;
  EXPECT_EQ(
    ekf.updateGyro(0, 0.01, Eigen::Vector3d(500.0, 0, 0), isoCov(0.01)),
    UpdateStatus::kRejectedNis);
  EXPECT_GT(ekf.lastNis(), 16.27);
  EXPECT_EQ(ekf.state().w, w_before);
  EXPECT_EQ(ekf.counters(0).rejected_nis, 1u);
}

TEST(Update, StepChangeDoesNotLockTheStreamOut)
{
  EkfConfig c = testConfig();
  c.nis_max_consecutive = 5;
  ImuEkf ekf(c);
  ekf.initialize(NominalState{}, 0.0);
  // The true rate jumps to 0.5 rad/s, far outside the noise model.
  int first_applied = -1;
  for (int k = 1; k <= 30; ++k) {
    if (ekf.updateGyro(1, 0.01 * k, Eigen::Vector3d(0.5, 0, 0), isoCov(0.01)) ==
      UpdateStatus::kApplied && first_applied < 0)
    {
      first_applied = k;
    }
  }
  ASSERT_GT(first_applied, 0);
  EXPECT_LE(first_applied, 6);  // 5 rejections, then forced
  EXPECT_GE(ekf.counters(1).forced, 1u);
  EXPECT_GT(ekf.state().w.x(), 0.2);
}

TEST(Update, SaturatedSampleIsRejected)
{
  EkfConfig c = testConfig();
  c.gyro_saturation = {10.0, 10.0};
  ImuEkf ekf(c);
  ekf.initialize(NominalState{}, 0.0);
  EXPECT_EQ(
    ekf.updateGyro(0, 0.01, Eigen::Vector3d(0, 0, 11.0), isoCov(0.01)),
    UpdateStatus::kRejectedSaturated);
  EXPECT_EQ(ekf.counters(0).rejected_saturated, 1u);
}

TEST(Update, StuckStreamIsRejectedAfterRepeatedIdenticalSamples)
{
  EkfConfig c = testConfig();
  c.stuck_count = 3;
  ImuEkf ekf(c);
  ekf.initialize(NominalState{}, 0.0);
  const Eigen::Vector3d z(0.01, 0.02, 0.03);
  UpdateStatus last = UpdateStatus::kApplied;
  for (int k = 1; k <= 5; ++k) {
    last = ekf.updateGyro(0, 0.01 * k, z, isoCov(0.01));
  }
  EXPECT_EQ(last, UpdateStatus::kRejectedStuck);
  EXPECT_GE(ekf.counters(0).rejected_stuck, 1u);
  // A different sample clears it.
  EXPECT_EQ(
    ekf.updateGyro(0, 0.07, Eigen::Vector3d(0.02, 0.02, 0.03), isoCov(0.01)),
    UpdateStatus::kApplied);
}

// Stationary rover, ICM gyro bias unknown, ZED already bias corrected (tight
// prior): the ICM bias must be learned from the difference between the two.
TEST(Update, IcmGyroBiasConvergesWhenZedBiasIsTight)
{
  EkfConfig c = testConfig();
  c.init_gyro_bias_sigma = {0.05, 0.0005};
  c.rate_walk = 0.01;
  ImuEkf ekf(c);
  ekf.initialize(NominalState{}, 0.0);

  const Eigen::Vector3d true_bias(0.02, -0.01, 0.015);
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0.0, 1.0);
  const double sigma = 0.01;
  for (int k = 1; k <= 3000; ++k) {
    const double t = 0.01 * k;
    const Eigen::Vector3d noise0(n(rng), n(rng), n(rng));
    const Eigen::Vector3d noise1(n(rng), n(rng), n(rng));
    ekf.updateGyro(1, t, sigma * noise1, isoCov(sigma));
    if (k % 2 == 0) {
      ekf.updateGyro(0, t, true_bias + sigma * noise0, isoCov(sigma));
    }
  }
  EXPECT_LT((ekf.state().bg[0] - true_bias).norm(), 0.004);
  EXPECT_LT(ekf.state().bg[1].norm(), 0.002);
}

TEST(Update, ConsistencyGyroNisAveragesMeasurementDimension)
{
  EkfConfig c = testConfig();
  c.rate_walk = 0.05;
  c.gyro_bias_walk = {1e-6, 1e-6};
  c.init_gyro_bias_sigma = {0.01, 0.001};
  ImuEkf ekf(c);

  const Eigen::Vector3d bias0(0.01, -0.02, 0.005);
  NominalState s;
  s.bg[0] = bias0;
  ekf.initialize(s, 0.0);

  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, 1.0);
  const double sigma = 0.02;
  const double dt = 0.01;
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  double nis_sum = 0.0;
  int nis_count = 0;
  for (int k = 1; k <= 12000; ++k) {
    const double t = dt * k;
    w += c.rate_walk * std::sqrt(dt) * Eigen::Vector3d(n(rng), n(rng), n(rng));
    const Eigen::Vector3d noise(n(rng), n(rng), n(rng));
    // IMU 1 at 100 Hz, IMU 0 at 50 Hz: asynchronous, different rates.
    const Eigen::Vector3d z1 = w + sigma * noise;
    ASSERT_NE(ekf.updateGyro(1, t, z1, isoCov(sigma)), UpdateStatus::kStale);
    if (k > 2000) {
      nis_sum += ekf.lastNis();
      ++nis_count;
    }
    if (k % 2 == 0) {
      const Eigen::Vector3d z0 = w + bias0 + sigma * Eigen::Vector3d(n(rng), n(rng), n(rng));
      ASSERT_NE(ekf.updateGyro(0, t, z0, isoCov(sigma)), UpdateStatus::kStale);
      if (k > 2000) {
        nis_sum += ekf.lastNis();
        ++nis_count;
      }
    }
  }
  EXPECT_NEAR(nis_sum / nis_count, 3.0, 0.3);
}

TEST(Update, ConsistencyAccelNisAveragesMeasurementDimension)
{
  EkfConfig c = testConfig();
  c.accel_walk = 0.5;
  c.accel_bias_walk = {1e-6, 1e-6};
  c.init_accel_bias_sigma = {0.01, 0.001};
  ImuEkf ekf(c);
  NominalState s;
  s.a = Eigen::Vector3d(0, 0, 9.8);
  ekf.initialize(s, 0.0);

  std::mt19937 rng(11);
  std::normal_distribution<double> n(0.0, 1.0);
  const double sigma = 0.1;
  const double dt = 0.01;
  Eigen::Vector3d a = s.a;
  double nis_sum = 0.0;
  int nis_count = 0;
  for (int k = 1; k <= 12000; ++k) {
    const double t = dt * k;
    a += c.accel_walk * std::sqrt(dt) * Eigen::Vector3d(n(rng), n(rng), n(rng));
    const Eigen::Vector3d z = a + sigma * Eigen::Vector3d(n(rng), n(rng), n(rng));
    ASSERT_NE(ekf.updateAccel(1, t, z, isoCov(sigma)), UpdateStatus::kStale);
    if (k > 2000) {
      nis_sum += ekf.lastNis();
      ++nis_count;
    }
  }
  EXPECT_NEAR(nis_sum / nis_count, 3.0, 0.3);
}

// ---------------------------------------------------------------------------
// Gravity pseudo-measurement
// ---------------------------------------------------------------------------

TEST(Gravity, CorrectsRollAndPitchFromSpecificForce)
{
  EkfConfig c = testConfig();
  c.init_attitude_sigma = 0.3;
  ImuEkf ekf(c);

  const Eigen::Quaterniond q_true(
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()));
  NominalState s;  // attitude wrongly level
  s.a = gravityBody(q_true);
  ekf.initialize(s, 0.0);

  const double before = ekf.covariance()(0, 0);
  for (int k = 1; k <= 100; ++k) {
    ASSERT_EQ(ekf.updateGravity(0.1 * k), UpdateStatus::kApplied);
  }
  const Eigen::Vector3d g_est = gravityBody(ekf.state().q);
  const Eigen::Vector3d g_true = gravityBody(q_true);
  EXPECT_LT(
    std::acos(std::min(1.0, g_est.normalized().dot(g_true.normalized()))), 6e-3);
  EXPECT_LT(ekf.covariance()(0, 0), before);
}

TEST(Gravity, SkippedUnderLargeDynamicAcceleration)
{
  ImuEkf ekf(testConfig());
  NominalState s;
  s.a = Eigen::Vector3d(0, 0, 9.80665 + 3.0);
  ekf.initialize(s, 0.0);
  const auto q_before = ekf.state().q;
  EXPECT_EQ(ekf.updateGravity(0.1), UpdateStatus::kSkipped);
  EXPECT_TRUE(ekf.state().q.isApprox(q_before, 1e-15));
  EXPECT_EQ(ekf.gravitySkipped(), 1u);
}

TEST(Gravity, NoiseGrowsWithDynamicAcceleration)
{
  EkfConfig c = testConfig();
  c.init_attitude_sigma = 0.3;
  ImuEkf calm(c);
  ImuEkf busy(c);
  NominalState s;
  s.a = Eigen::Vector3d(0, 0, 9.80665);
  calm.initialize(s, 0.0);
  s.a = Eigen::Vector3d(0, 0, 9.80665 + 1.0);  // |a| - g = 1 m/s^2
  busy.initialize(s, 0.0);
  ASSERT_EQ(calm.updateGravity(0.1), UpdateStatus::kApplied);
  ASSERT_EQ(busy.updateGravity(0.1), UpdateStatus::kApplied);
  const double calm_reduction = c.init_attitude_sigma * c.init_attitude_sigma -
    calm.covariance()(0, 0);
  const double busy_reduction = c.init_attitude_sigma * c.init_attitude_sigma -
    busy.covariance()(0, 0);
  EXPECT_GT(calm_reduction, busy_reduction);
}

// ---------------------------------------------------------------------------
// Magnetometer yaw
// ---------------------------------------------------------------------------

TEST(MagYaw, FirstCallSetsReferenceThenYawIsCorrected)
{
  EkfConfig c = testConfig();
  c.init_attitude_sigma = 0.5;
  ImuEkf ekf(c);
  ekf.initialize(NominalState{}, 0.0);

  const Eigen::Vector3d field(1.0, 0.0, 0.5);  // fixed in the world
  EXPECT_EQ(ekf.updateMagYaw(0.01, field), UpdateStatus::kSkipped);
  EXPECT_TRUE(ekf.hasMagReference());

  // The body actually yaws by 0.3 rad; the filter was not told.
  const Eigen::Vector3d m_body =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix().transpose() * field;
  const double before = ekf.orientationCovarianceWorld()(2, 2);
  for (int k = 2; k <= 40; ++k) {
    ekf.updateMagYaw(0.01 * k, m_body);
  }
  const double yaw = std::atan2(
    ekf.state().q.toRotationMatrix()(1, 0), ekf.state().q.toRotationMatrix()(0, 0));
  EXPECT_NEAR(yaw, 0.3, 2e-3);
  EXPECT_LT(ekf.orientationCovarianceWorld()(2, 2), before);
}

TEST(MagYaw, ThrowsWhenSigmaNotSet)
{
  EkfConfig c = testConfig();
  c.mag_yaw_sigma = -1.0;
  ImuEkf ekf(c);
  ekf.initialize(NominalState{}, 0.0);
  EXPECT_THROW(ekf.updateMagYaw(0.1, Eigen::Vector3d(1, 0, 0)), std::logic_error);
}
