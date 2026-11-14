#pragma once

#include <Eigen/Dense>

#include <array>
#include <cstddef>

namespace rover_imu_fusion
{

// Error-state EKF for the rover IMUs. No ROS dependency.
//
// Frames
//   world : fixed, gravity along +z. Yaw is relative (arbitrary start heading).
//   body  : base_link. q rotates body vectors into the world: v_w = R(q) v_b.
//
// Nominal state
//   q            orientation (world <- body)
//   w            body angular rate (rad/s)
//   a            body specific force (m/s^2), i.e. includes gravity
//   bg[i], ba[i] gyro and accel bias of IMU i (i = 0 ICM, 1 ZED), expressed in
//                base_link axes (measurements are rotated into base_link
//                before they reach the filter, which rotates the bias too)
//
// Error state (dimension 21), ordering used for P, F, H and Q:
//   [  0: 3)  dtheta   attitude error, body frame:  q_true = q * Exp(dtheta)
//   [  3: 6)  dw
//   [  6: 9)  da
//   [  9:12)  dbg0     [12:15)  dba0
//   [ 15:18)  dbg1     [18:21)  dba1
//
// Process model
//   q   <- q * Exp(w dt)
//   w, a, biases: random walks (unchanged in the nominal state)
//   F : dtheta' = Exp(-w dt) dtheta + dt (I - 1/2 [w dt]x) dw ; rest identity
//   Q : diagonal, sigma^2 dt on the w, a and bias blocks
//
// Both IMUs are measurements, not the propagation input. With
//   h_gyro_i  = w + bg_i        H = [0 I 0 .. I at dbg_i ..]
//   h_accel_i = a + ba_i        H = [0 0 I .. I at dba_i ..]
// the update is the standard EKF update (Joseph form), followed by error
// injection (q = q * Exp(dtheta), others additive) and the covariance reset
// P <- G P G^T with G = blockdiag(I - 1/2 [dtheta]x, I, ...).
//
// Gravity pseudo-measurement (roll/pitch): a = R(q)^T g_vec, g_vec = (0, 0, g).
//   Using R(q * Exp(dtheta))^T g = R^T g + [R^T g]x dtheta
//   y = R^T g_vec - a ,  H = [-[R^T g_vec]x, 0, I, 0 ...]
//   noise sigma_g^2 (1 + (( |a| - g ) / adapt)^2); skipped if ||a| - g| > skip.
//
// Magnetometer yaw (optional): with m_w = R m_b and heading alpha = atan2(m_wy, m_wx)
//   y = wrap(alpha_ref - alpha), H_theta = (R^T e_z)^T, noise sigma_m^2.
//   alpha_ref is the heading at the first sample, so yaw stays relative to
//   the start heading (no magnetic north, no declination).
constexpr int kStateDim = 21;
using StateCov = Eigen::Matrix<double, kStateDim, kStateDim>;
using StateVec = Eigen::Matrix<double, kStateDim, 1>;

namespace idx
{
constexpr int kTheta = 0;
constexpr int kRate = 3;
constexpr int kAccel = 6;
constexpr int gyroBias(int imu) {return 9 + 6 * imu;}
constexpr int accelBias(int imu) {return 12 + 6 * imu;}
}  // namespace idx

// Skew-symmetric matrix: skew(a) * b = a x b.
Eigen::Matrix3d skew(const Eigen::Vector3d & v);

// Rotation vector (rad) to unit quaternion, and back.
Eigen::Quaterniond expQuat(const Eigen::Vector3d & rotvec);
Eigen::Vector3d logQuat(const Eigen::Quaterniond & q);

struct NominalState
{
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 2> bg{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 2> ba{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
};

// Noise values have no defaults (-1 = unset) on purpose so a filter cannot be
// built that silently assumes a perfect sensor. Process noise is per
// sqrt(second): the added variance over dt is sigma^2 * dt.
struct EkfConfig
{
  // Process noise (random walk strengths).
  double rate_walk{-1.0};                              // rad/s / sqrt(s)
  double accel_walk{-1.0};                             // m/s^2 / sqrt(s)
  std::array<double, 2> gyro_bias_walk{-1.0, -1.0};    // rad/s / sqrt(s)
  std::array<double, 2> accel_bias_walk{-1.0, -1.0};   // m/s^2 / sqrt(s)

  // Initial uncertainty.
  double init_attitude_sigma{-1.0};                         // rad
  double init_rate_sigma{-1.0};                             // rad/s
  double init_accel_sigma{-1.0};                            // m/s^2
  std::array<double, 2> init_gyro_bias_sigma{-1.0, -1.0};   // rad/s
  std::array<double, 2> init_accel_bias_sigma{-1.0, -1.0};  // m/s^2

  // Gravity pseudo-measurement. gravity_sigma is the expected non-gravity
  // acceleration (vibration, driving) and must be set.
  double gravity{9.80665};     // m/s^2
  double gravity_sigma{-1.0};  // m/s^2
  double gravity_adapt{0.5};   // m/s^2, noise doubles when ||a| - g| equals this
  double gravity_skip{2.0};    // m/s^2, update skipped beyond this

  // Magnetometer yaw noise (rad). Only needed when the magnetometer is used.
  double mag_yaw_sigma{-1.0};

  // NIS (chi-square) gates. Defaults are the 99.9 % points.
  double nis_gate_3dof{16.27};
  double nis_gate_1dof{10.83};

  // After this many consecutive NIS rejections on one IMU stream the next
  // sample is applied regardless of the gate, so a real step change (or a bad
  // model) cannot lock the stream out. 0 disables the recovery.
  int nis_max_consecutive{20};

  // Saturation limits per IMU (norm, rad/s and m/s^2); 0 disables the check.
  std::array<double, 2> gyro_saturation{0.0, 0.0};
  std::array<double, 2> accel_saturation{0.0, 0.0};
  // Identical consecutive samples that mark a stream as stuck; 0 disables.
  int stuck_count{0};

  // Throws std::invalid_argument on missing or invalid values.
  void validate() const;
};

enum class UpdateStatus
{
  kApplied,
  kStale,              // measurement older than the filter time
  kRejectedNis,
  kRejectedSaturated,
  kRejectedStuck,
  kSkipped,            // gravity update skipped (dynamic acceleration) or no mag reference
};

// Per-IMU bookkeeping for diagnostics.
struct ImuCounters
{
  std::size_t applied{0};
  std::size_t stale{0};
  std::size_t rejected_nis{0};
  std::size_t rejected_saturated{0};
  std::size_t rejected_stuck{0};
  std::size_t forced{0};    // samples applied through the NIS gate by the recovery rule
  double last_time{-1.0};   // time of the last non-stale sample, -1 if none

  std::size_t rejected() const
  {
    return stale + rejected_nis + rejected_saturated + rejected_stuck;
  }
};

class ImuEkf
{
public:
  explicit ImuEkf(const EkfConfig & config);

  // Sets the nominal state and the initial covariance from the config.
  void initialize(const NominalState & state, double time);

  bool initialized() const {return initialized_;}
  double time() const {return time_;}
  const NominalState & state() const {return x_;}
  const StateCov & covariance() const {return P_;}

  // Propagate the state and covariance to absolute time t (seconds).
  // Returns false and changes nothing if t is earlier than the filter time.
  bool predict(double t);

  // Measurement updates. Each one first propagates to time t; a stale t
  // returns kStale. z and R are in base_link axes.
  UpdateStatus updateGyro(int imu, double t, const Eigen::Vector3d & z, const Eigen::Matrix3d & R);
  UpdateStatus updateAccel(int imu, double t, const Eigen::Vector3d & z, const Eigen::Matrix3d & R);
  UpdateStatus updateGravity(double t);

  // Magnetometer yaw. The first call stores the current heading as the
  // reference and returns kSkipped; later calls update yaw. m_body is the
  // calibrated field in base_link axes (any units).
  UpdateStatus updateMagYaw(double t, const Eigen::Vector3d & m_body);
  bool hasMagReference() const {return has_mag_ref_;}

  const ImuCounters & counters(int imu) const;
  std::size_t gravityApplied() const {return gravity_applied_;}
  std::size_t gravitySkipped() const {return gravity_skipped_;}
  // NIS of the last update that reached the gate (applied or rejected).
  double lastNis() const {return last_nis_;}

  // Discrete error-state transition F for a step of dt at the current state.
  StateCov transition(double dt) const;

  // Attitude covariance as rotation about fixed world axes (REP-145):
  // R P_theta R^T. The world yaw variance is element (2, 2).
  Eigen::Matrix3d orientationCovarianceWorld() const;

private:
  struct StreamHealth
  {
    Eigen::Vector3d last{Eigen::Vector3d::Zero()};
    int repeats{0};
    bool has_last{false};
  };

  EkfConfig cfg_;
  NominalState x_;
  StateCov P_{StateCov::Zero()};
  double time_{0.0};
  bool initialized_{false};

  std::array<ImuCounters, 2> counters_;
  std::array<StreamHealth, 2> gyro_health_;
  std::array<StreamHealth, 2> accel_health_;
  std::array<int, 2> gyro_consec_nis_{0, 0};
  std::array<int, 2> accel_consec_nis_{0, 0};
  std::size_t gravity_applied_{0};
  std::size_t gravity_skipped_{0};
  double last_nis_{0.0};

  double mag_ref_heading_{0.0};
  bool has_mag_ref_{false};

  // Returns kApplied when the sample is healthy, otherwise the reject reason.
  UpdateStatus checkHealth(StreamHealth & h, const Eigen::Vector3d & z, double limit) const;

  template<int M>
  UpdateStatus applyUpdate(
    const Eigen::Matrix<double, M, kStateDim> & H, const Eigen::Matrix<double, M, 1> & y,
    const Eigen::Matrix<double, M, M> & R, double gate);

  UpdateStatus vectorUpdate(
    int imu, double t, const Eigen::Vector3d & z, const Eigen::Matrix3d & R, bool gyro);
  void inject(const StateVec & dx);
};

}  // namespace rover_imu_fusion
