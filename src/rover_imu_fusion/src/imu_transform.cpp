#include "rover_imu_fusion/imu_transform.hpp"

namespace rover_imu_fusion
{

namespace
{

// Relative tolerances for the symmetry and PSD checks. Message covariances
// are float-derived (ZED SDK), so exact equality is too strict.
constexpr double kSymmetryTol = 1e-6;
constexpr double kPsdTol = 1e-9;

}  // namespace

Eigen::Matrix3d covarianceFromArray(const std::array<double, 9> & a)
{
  Eigen::Matrix3d m;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      m(r, c) = a[static_cast<std::size_t>(r * 3 + c)];
    }
  }
  return m;
}

std::array<double, 9> covarianceToArray(const Eigen::Matrix3d & m)
{
  std::array<double, 9> a{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      a[static_cast<std::size_t>(r * 3 + c)] = m(r, c);
    }
  }
  return a;
}

bool isCovarianceProvided(const Eigen::Matrix3d & cov)
{
  if (!cov.allFinite()) {
    return false;
  }
  if (cov.isZero(0.0)) {
    return false;
  }
  // REP-145: first element -1 means the measurement is not available.
  if (cov(0, 0) == -1.0) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (cov(i, i) <= 0.0) {
      return false;
    }
  }
  const double scale = cov.cwiseAbs().maxCoeff();
  if ((cov - cov.transpose()).cwiseAbs().maxCoeff() > kSymmetryTol * scale) {
    return false;
  }
  const Eigen::Matrix3d sym = 0.5 * (cov + cov.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(sym, Eigen::EigenvaluesOnly);
  return eig.eigenvalues().minCoeff() >= -kPsdTol * scale;
}

Eigen::Matrix3d rotateCovariance(const Eigen::Matrix3d & R, const Eigen::Matrix3d & cov)
{
  const Eigen::Matrix3d out = R * cov * R.transpose();
  return 0.5 * (out + out.transpose());
}

Eigen::Quaterniond baseOrientation(
  const Eigen::Quaterniond & q_world_imu, const Eigen::Matrix3d & R_base_imu)
{
  Eigen::Quaterniond q = q_world_imu * Eigen::Quaterniond(R_base_imu.transpose());
  q.normalize();
  return q;
}

}  // namespace rover_imu_fusion
