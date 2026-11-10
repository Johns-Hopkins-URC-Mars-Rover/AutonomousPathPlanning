#pragma once

#include <Eigen/Dense>

#include <array>

namespace rover_imu_fusion
{

// Row-major 9-array (ROS covariance layout) to Eigen matrix.
Eigen::Matrix3d covarianceFromArray(const std::array<double, 9> & a);

// Eigen matrix to row-major 9-array.
std::array<double, 9> covarianceToArray(const Eigen::Matrix3d & m);

// True when a message covariance can be trusted as a real covariance.
// "Not provided" cases: all zero, first element -1 (REP-145 "unknown"),
// non-finite entries, non-positive diagonal, not symmetric, or not
// positive semi-definite.
bool isCovarianceProvided(const Eigen::Matrix3d & cov);

// Covariance of R * v when v has covariance cov: R cov R^T.
Eigen::Matrix3d rotateCovariance(const Eigen::Matrix3d & R, const Eigen::Matrix3d & cov);

// Orientation of the base frame in the world frame.
//   q_world_imu : orientation reported by the IMU (world <- imu)
//   R_base_imu  : rotation taking IMU-frame vectors to base-frame vectors
// Result is q_world_imu * R_base_imu^T, i.e. world <- base.
// A rotation covariance expressed about fixed (world) axes is unchanged by
// this right-multiplication.
Eigen::Quaterniond baseOrientation(
  const Eigen::Quaterniond & q_world_imu, const Eigen::Matrix3d & R_base_imu);

}  // namespace rover_imu_fusion
