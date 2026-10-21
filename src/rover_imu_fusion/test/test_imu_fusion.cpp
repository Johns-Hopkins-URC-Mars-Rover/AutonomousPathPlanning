#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "rover_imu_fusion/imu_fusion.hpp"
#include "rover_imu_fusion/imu_initializer.hpp"
#include "rover_imu_fusion/reorder_buffer.hpp"

using rover_imu_fusion::FusionConfig;
using rover_imu_fusion::ImuFusion;
using rover_imu_fusion::ImuInitializer;
using rover_imu_fusion::InitConfig;
using rover_imu_fusion::logQuat;
using rover_imu_fusion::Measurement;
using rover_imu_fusion::ReorderBuffer;
namespace idx = rover_imu_fusion::idx;

namespace
{

constexpr double kG = 9.80665;

FusionConfig testConfig()
{
  FusionConfig c;
  c.ekf.rate_walk = 0.02;
  c.ekf.accel_walk = 0.3;
  c.ekf.gyro_bias_walk = {1e-5, 1e-5};
  c.ekf.accel_bias_walk = {1e-4, 1e-4};
  c.ekf.init_attitude_sigma = 0.01;
  c.ekf.init_rate_sigma = 0.01;
  c.ekf.init_accel_sigma = 0.1;
  c.ekf.init_gyro_bias_sigma = {0.01, 0.002};
  c.ekf.init_accel_bias_sigma = {0.05, 0.05};
  c.ekf.gravity_sigma = 0.3;
  return c;
}

Eigen::Vector3d gravityBody(const Eigen::Quaterniond & q)
{
  return q.toRotationMatrix().transpose() * Eigen::Vector3d(0, 0, kG);
}

// Synthetic rover: known true motion, two IMUs with known bias and noise.
class Sim
{
public:
  Sim(unsigned seed, double gyro_sigma, double accel_sigma)
  : rng_(seed), gyro_sigma_(gyro_sigma), accel_sigma_(accel_sigma) {}

  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 2> gyro_bias{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 2> accel_bias{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

  // Advance truth by dt, then emit one sample of each type for the IMU.
  void advance(double dt) {q = (q * rover_imu_fusion::expQuat(w * dt)).normalized();}

  Measurement gyro(int imu, double t)
  {
    Measurement m;
    m.kind = Measurement::Kind::kGyro;
    m.imu = imu;
    m.stamp = t;
    m.z = w + gyro_bias[static_cast<std::size_t>(imu)] + noise(gyro_sigma_);
    m.R = gyro_sigma_ * gyro_sigma_ * Eigen::Matrix3d::Identity();
    return m;
  }

  Measurement accel(int imu, double t)
  {
    Measurement m;
    m.kind = Measurement::Kind::kAccel;
    m.imu = imu;
    m.stamp = t;
    m.z = gravityBody(q) + accel_bias[static_cast<std::size_t>(imu)] + noise(accel_sigma_);
    m.R = accel_sigma_ * accel_sigma_ * Eigen::Matrix3d::Identity();
    return m;
  }

private:
  std::mt19937 rng_;
  std::normal_distribution<double> n_{0.0, 1.0};
  double gyro_sigma_;
  double accel_sigma_;

  Eigen::Vector3d noise(double sigma)
  {
    return sigma * Eigen::Vector3d(n_(rng_), n_(rng_), n_(rng_));
  }
};

// Runs the pipeline in 10 ms ticks: IMU 1 (ZED-like) every tick, IMU 0
// (ICM-like) every other tick, then process(now - delay).
void run(ImuFusion & f, Sim & sim, double t0, double t1, bool icm_present, double delay = 0.015)
{
  const double dt = 0.01;
  int k = 0;
  for (double t = t0; t < t1 - 1e-12; t += dt, ++k) {
    sim.advance(dt);
    const double now = t + dt;
    f.add(sim.gyro(1, now));
    f.add(sim.accel(1, now));
    if (icm_present && k % 2 == 0) {
      f.add(sim.gyro(0, now));
      f.add(sim.accel(0, now));
    }
    f.process(now - delay);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// ReorderBuffer
// ---------------------------------------------------------------------------

TEST(ReorderBuffer, ReleasesInStampOrderUpToCutoff)
{
  ReorderBuffer<int> b;
  b.push(0.3, 3);
  b.push(0.1, 1);
  b.push(0.2, 2);
  b.push(0.5, 5);
  const auto out = b.popUntil(0.3);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0], 1);
  EXPECT_EQ(out[1], 2);
  EXPECT_EQ(out[2], 3);
  EXPECT_EQ(b.size(), 1u);
  EXPECT_TRUE(b.popUntil(0.4).empty());
}

TEST(ReorderBuffer, DropsOldestWhenFull)
{
  ReorderBuffer<int> b(2);
  b.push(1.0, 1);
  b.push(2.0, 2);
  b.push(3.0, 3);
  EXPECT_EQ(b.dropped(), 1u);
  const auto out = b.popUntil(10.0);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0], 2);
}

// ---------------------------------------------------------------------------
// ImuInitializer
// ---------------------------------------------------------------------------

TEST(Initializer, SeedsRollPitchAndGyroBiasFromStationaryData)
{
  InitConfig c;
  ImuInitializer init(c);
  const Eigen::Quaterniond q_true(
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d bias(0.02, -0.01, 0.015);
  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int k = 0; k < 150 && !init.ready(); ++k) {
    const double t = 0.01 * k;
    init.addGyro(0, t, bias + 0.005 * Eigen::Vector3d(n(rng), n(rng), n(rng)));
    init.addAccel(0, t, gravityBody(q_true) + 0.05 * Eigen::Vector3d(n(rng), n(rng), n(rng)));
  }
  ASSERT_TRUE(init.ready());
  EXPECT_GE(init.time(), 1.0);
  EXPECT_LT((init.state().bg[0] - bias).norm(), 0.005);
  EXPECT_TRUE(init.state().bg[1].isZero(0.0));
  const Eigen::Vector3d g_est = gravityBody(init.state().q);
  EXPECT_LT((g_est - gravityBody(q_true)).norm(), 0.05);
}

TEST(Initializer, RestartsWhenMoving)
{
  InitConfig c;
  ImuInitializer init(c);
  std::mt19937 rng(5);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int k = 0; k < 110; ++k) {  // violent shaking
    const double t = 0.01 * k;
    init.addGyro(0, t, 2.0 * Eigen::Vector3d(n(rng), n(rng), n(rng)));
    init.addAccel(0, t, Eigen::Vector3d(0, 0, kG) + 3.0 * Eigen::Vector3d(n(rng), n(rng), n(rng)));
  }
  EXPECT_FALSE(init.ready());
  EXPECT_GE(init.restarts(), 1u);
}

TEST(Initializer, SteadyRotationIsNotMistakenForBias)
{
  InitConfig c;
  ImuInitializer init(c);
  for (int k = 0; k < 150; ++k) {  // smooth 0.3 rad/s turn, no noise
    init.addGyro(0, 0.01 * k, Eigen::Vector3d(0, 0, 0.3));
    init.addGyro(1, 0.01 * k, Eigen::Vector3d(0, 0, 0.3));
    init.addAccel(0, 0.01 * k, Eigen::Vector3d(0, 0, kG));
  }
  EXPECT_FALSE(init.ready());
  EXPECT_GE(init.restarts(), 1u);
}

TEST(Initializer, ARealGyroBiasOnOneImuStillInitializes)
{
  InitConfig c;
  ImuInitializer init(c);
  for (int k = 0; k < 150 && !init.ready(); ++k) {
    init.addGyro(0, 0.01 * k, Eigen::Vector3d(0.08, -0.05, 0.03));  // ICM-like bias
    init.addGyro(1, 0.01 * k, Eigen::Vector3d::Zero());             // ZED, corrected
    init.addAccel(0, 0.01 * k, Eigen::Vector3d(0, 0, kG));
  }
  EXPECT_TRUE(init.ready());
}

TEST(Initializer, RejectsAccelThatIsNotGravity)
{
  InitConfig c;
  ImuInitializer init(c);
  for (int k = 0; k < 110; ++k) {
    init.addGyro(0, 0.01 * k, Eigen::Vector3d::Zero());
    init.addAccel(0, 0.01 * k, Eigen::Vector3d(0, 0, 3.0));  // free fall
  }
  EXPECT_FALSE(init.ready());
}

// ---------------------------------------------------------------------------
// ImuFusion pipeline
// ---------------------------------------------------------------------------

TEST(Fusion, DoesNotPublishBeforeInitialized)
{
  ImuFusion f(testConfig());
  Sim sim(1, 0.01, 0.1);
  run(f, sim, 0.0, 0.5, true);
  EXPECT_FALSE(f.initialized());
  EXPECT_FALSE(f.output().has_value());
  run(f, sim, 0.5, 2.0, true);
  EXPECT_TRUE(f.initialized());
  EXPECT_TRUE(f.output().has_value());
}

TEST(Fusion, StationaryRollPitchStayFlatAndYawVarianceGrows)
{
  ImuFusion f(testConfig());
  Sim sim(2, 0.01, 0.1);
  sim.gyro_bias[0] = Eigen::Vector3d(0.02, -0.01, 0.015);
  sim.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitX()));

  run(f, sim, 0.0, 2.0, true);
  ASSERT_TRUE(f.initialized());
  const Eigen::Quaterniond q_start = f.output()->q;
  double prev_yaw_var = f.output()->q_cov(2, 2);
  double max_tilt_err = 0.0;

  for (int s = 2; s < 62; ++s) {
    run(f, sim, s * 1.0, (s + 1) * 1.0, true);
    const auto out = f.output();
    const Eigen::Vector3d g_est = gravityBody(out->q);
    const Eigen::Vector3d g_true = gravityBody(sim.q);
    max_tilt_err = std::max(
      max_tilt_err,
      std::acos(std::min(1.0, g_est.normalized().dot(g_true.normalized()))));
    ASSERT_GT(out->q_cov(2, 2), prev_yaw_var) << "second " << s;
    prev_yaw_var = out->q_cov(2, 2);
  }
  EXPECT_LT(max_tilt_err, 0.01);
  // Yaw drift stays small for a still rover with a corrected gyro bias.
  EXPECT_LT(logQuat(q_start.inverse() * f.output()->q).norm(), 0.05);
}

TEST(Fusion, ConstantRotationIntegratesAboutEachAxis)
{
  for (int axis = 0; axis < 3; ++axis) {
    // The rate state must be allowed to change as fast as a turn starts.
    FusionConfig cfg = testConfig();
    cfg.ekf.rate_walk = 1.0;
    ImuFusion f(cfg);
    Sim sim(10 + axis, 0.01, 0.1);
    run(f, sim, 0.0, 2.0, true);  // initialize while still
    ASSERT_TRUE(f.initialized());
    const Eigen::Quaterniond q_start_est = f.output()->q;
    const Eigen::Quaterniond q_start_true = sim.q;

    Eigen::Vector3d unit = Eigen::Vector3d::Zero();
    unit(axis) = 1.0;
    sim.w = 0.5 * unit;
    run(f, sim, 2.0, 4.0, true);  // 1 rad about one axis
    const Eigen::Quaterniond dq_est = q_start_est.inverse() * f.output()->q;
    const Eigen::Quaterniond dq_true = q_start_true.inverse() * sim.q;
    EXPECT_LT(logQuat(dq_est.inverse() * dq_true).norm(), 0.05) << "axis " << axis;
    // The estimate lags the truth by the reorder delay (15 ms at 0.5 rad/s).
  }
}

TEST(Fusion, OneImuRemovedMidRunKeepsOutputAndWidensItsCovariance)
{
  // Bias random walks large enough to show up over the 14 s dropout.
  FusionConfig cfg = testConfig();
  cfg.ekf.gyro_bias_walk = {1e-3, 1e-5};
  cfg.ekf.accel_bias_walk = {3e-2, 1e-4};
  ImuFusion f(cfg);
  Sim sim(4, 0.01, 0.1);
  sim.gyro_bias[0] = Eigen::Vector3d(0.02, -0.01, 0.015);

  run(f, sim, 0.0, 6.0, true);
  ASSERT_TRUE(f.initialized());
  const double stamp_before = f.output()->stamp;
  const double bg0_var_before = f.filter()->covariance()(idx::gyroBias(0), idx::gyroBias(0));
  const double bg1_var_before = f.filter()->covariance()(idx::gyroBias(1), idx::gyroBias(1));
  const double ba0_var_before = f.filter()->covariance()(idx::accelBias(0), idx::accelBias(0));

  run(f, sim, 6.0, 20.0, false);  // ICM gone
  const auto out = f.output();
  ASSERT_TRUE(out.has_value());
  EXPECT_GT(out->stamp, stamp_before + 13.0);               // output keeps running
  EXPECT_TRUE(f.timedOut(0, 20.0));
  EXPECT_FALSE(f.timedOut(1, 20.0));
  // The silent IMU's bias covariance widens ...
  EXPECT_GT(
    f.filter()->covariance()(idx::gyroBias(0), idx::gyroBias(0)), 1.5 * bg0_var_before);
  EXPECT_GT(
    f.filter()->covariance()(idx::accelBias(0), idx::accelBias(0)), 1.5 * ba0_var_before);
  // ... while the remaining IMU's bias stays tight.
  EXPECT_LT(
    f.filter()->covariance()(idx::gyroBias(1), idx::gyroBias(1)), 5.0 * bg1_var_before);
  // And the tilt estimate is still good.
  const Eigen::Vector3d g_est = gravityBody(out->q);
  EXPECT_LT(
    std::acos(std::min(1.0, g_est.normalized().dot(gravityBody(sim.q).normalized()))), 0.02);
}

TEST(Fusion, WorksWithSingleImuFromTheStart)
{
  ImuFusion f(testConfig());
  Sim sim(6, 0.01, 0.1);
  run(f, sim, 0.0, 5.0, false);  // only IMU 1 ever reports
  ASSERT_TRUE(f.initialized());
  EXPECT_TRUE(f.timedOut(0, 5.0));
  EXPECT_FALSE(f.timedOut(1, 5.0));
}

TEST(Fusion, OutOfOrderArrivalIsPutInOrderByTheBuffer)
{
  ImuFusion f(testConfig());
  Sim sim(8, 0.01, 0.1);
  // Initialize first.
  run(f, sim, 0.0, 2.0, true);
  ASSERT_TRUE(f.initialized());
  // Deliver two samples swapped, within the reorder window.
  auto a = sim.gyro(1, 2.010);
  auto b = sim.gyro(1, 2.020);
  f.add(b);
  f.add(a);
  f.process(2.030);
  EXPECT_EQ(f.filter()->counters(1).stale, 0u);
}
