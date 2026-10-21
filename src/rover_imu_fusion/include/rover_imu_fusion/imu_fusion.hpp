#pragma once

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <optional>

#include "rover_imu_fusion/imu_ekf.hpp"
#include "rover_imu_fusion/imu_initializer.hpp"
#include "rover_imu_fusion/reorder_buffer.hpp"

namespace rover_imu_fusion
{

struct Measurement
{
  enum class Kind {kGyro, kAccel, kMag};
  Kind kind{Kind::kGyro};
  int imu{0};                                    // 0 ICM, 1 ZED (ignored for kMag)
  double stamp{0.0};                             // s
  Eigen::Vector3d z{Eigen::Vector3d::Zero()};    // base_link axes
  Eigen::Matrix3d R{Eigen::Matrix3d::Zero()};    // measurement covariance, base_link axes
};

struct FusionConfig
{
  EkfConfig ekf;
  InitConfig init;
  double reorder_delay{0.015};         // s measurements wait to be put in order
  double imu_timeout{0.5};             // s without a sample before an IMU is "timed out"
  double gravity_update_period{0.1};   // s between gravity pseudo-measurements
  std::size_t buffer_size{1000};
};

// Output in REP-145 conventions: acceleration includes gravity, covariances are
// the matching blocks of P (orientation about fixed world axes).
struct FusionOutput
{
  double stamp{0.0};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Matrix3d q_cov{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d w_cov{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d a_cov{Eigen::Matrix3d::Zero()};
};

// ROS-free pipeline: reorder buffer -> stationary initialization -> EKF.
// Both IMUs are measurements, so asynchronous rates and an IMU dropping out
// need no special handling.
class ImuFusion
{
public:
  explicit ImuFusion(const FusionConfig & config);

  // Queue a measurement. Stamps are in seconds on a common clock.
  void add(const Measurement & m);

  // Process every queued measurement with stamp <= cutoff (in stamp order),
  // then propagate the filter to the cutoff. Call periodically with
  // cutoff = now - reorder_delay.
  void process(double cutoff);

  bool initialized() const {return ekf_.has_value();}
  std::optional<FusionOutput> output() const;
  const ImuEkf * filter() const {return ekf_ ? &*ekf_ : nullptr;}

  // Newest sample stamp seen from the IMU, or -1 if none.
  double lastStamp(int imu) const
  {
    return (imu < 0 || imu > 1) ? -1.0 : last_stamp_[static_cast<std::size_t>(imu)];
  }

  // True if the IMU has produced no sample within imu_timeout of now.
  bool timedOut(int imu, double now) const;
  std::size_t rejectedBeforeInit() const {return init_.restarts();}
  std::size_t droppedFromBuffer() const {return buffer_.dropped();}

private:
  FusionConfig cfg_;
  ReorderBuffer<Measurement> buffer_;
  ImuInitializer init_;
  std::optional<ImuEkf> ekf_;
  std::array<double, 2> last_stamp_{-1.0, -1.0};
  double last_gravity_{-1e18};

  void handle(const Measurement & m);
};

}  // namespace rover_imu_fusion
