#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "rover_imu_fusion/imu_transform.hpp"

using rover_imu_fusion::baseOrientation;
using rover_imu_fusion::covarianceFromArray;
using rover_imu_fusion::covarianceToArray;
using rover_imu_fusion::isCovarianceProvided;
using rover_imu_fusion::rotateCovariance;

namespace
{

std::array<double, 9> diag(double a, double b, double c)
{
  return {a, 0.0, 0.0, 0.0, b, 0.0, 0.0, 0.0, c};
}

Eigen::Matrix3d rotZ(double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

}  // namespace

TEST(CovarianceProvided, AllZeroIsNotProvided)
{
  EXPECT_FALSE(isCovarianceProvided(Eigen::Matrix3d::Zero()));
}

TEST(CovarianceProvided, MinusOneMeansUnknown)
{
  EXPECT_FALSE(isCovarianceProvided(covarianceFromArray(diag(-1.0, 0.0, 0.0))));
}

TEST(CovarianceProvided, NonFiniteIsNotProvided)
{
  auto a = diag(1.0, 1.0, 1.0);
  a[4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isCovarianceProvided(covarianceFromArray(a)));
}

TEST(CovarianceProvided, ZeroOrNegativeDiagonalIsNotProvided)
{
  EXPECT_FALSE(isCovarianceProvided(covarianceFromArray(diag(1.0, 0.0, 1.0))));
  EXPECT_FALSE(isCovarianceProvided(covarianceFromArray(diag(1.0, 1.0, -1.0))));
}

TEST(CovarianceProvided, AsymmetricIsNotProvided)
{
  auto a = diag(1.0, 1.0, 1.0);
  a[1] = 0.5;
  EXPECT_FALSE(isCovarianceProvided(covarianceFromArray(a)));
}

TEST(CovarianceProvided, NotPositiveSemiDefiniteIsNotProvided)
{
  auto a = diag(1.0, 1.0, 1.0);
  a[1] = a[3] = 2.0;
  EXPECT_FALSE(isCovarianceProvided(covarianceFromArray(a)));
}

TEST(CovarianceProvided, ValidIsProvided)
{
  EXPECT_TRUE(isCovarianceProvided(covarianceFromArray(diag(1e-4, 2e-4, 3e-4))));
}

TEST(ArrayConversion, RoundTrips)
{
  const std::array<double, 9> a = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  EXPECT_EQ(covarianceToArray(covarianceFromArray(a)), a);
}

TEST(RotateCovariance, QuarterTurnAboutZSwapsXY)
{
  const Eigen::Matrix3d out =
    rotateCovariance(rotZ(M_PI / 2.0), covarianceFromArray(diag(1.0, 4.0, 9.0)));
  EXPECT_NEAR(out(0, 0), 4.0, 1e-12);
  EXPECT_NEAR(out(1, 1), 1.0, 1e-12);
  EXPECT_NEAR(out(2, 2), 9.0, 1e-12);
  EXPECT_NEAR(out(0, 1), 0.0, 1e-12);
}

TEST(RotateCovariance, PreservesTrace)
{
  const Eigen::Matrix3d c = covarianceFromArray(diag(1.0, 4.0, 9.0));
  const Eigen::Matrix3d R =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  EXPECT_NEAR(rotateCovariance(R, c).trace(), c.trace(), 1e-12);
}

TEST(BaseOrientation, IdentityMountingKeepsOrientation)
{
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(baseOrientation(q, Eigen::Matrix3d::Identity()).isApprox(q));
}

TEST(BaseOrientation, ConsistentWithVectorRotation)
{
  // A vector fixed in the base frame, expressed in the world frame, must be
  // the same whether reached through the IMU or through the base.
  const Eigen::Matrix3d R_bi = rotZ(M_PI / 2.0);
  const Eigen::Quaterniond q_wi(
    Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 1, 0).normalized()));
  const Eigen::Vector3d v_base(1.0, 2.0, 3.0);
  const Eigen::Vector3d v_imu = R_bi.transpose() * v_base;
  const Eigen::Vector3d via_imu = q_wi * v_imu;
  const Eigen::Vector3d via_base = baseOrientation(q_wi, R_bi) * v_base;
  EXPECT_TRUE(via_imu.isApprox(via_base, 1e-12));
}
