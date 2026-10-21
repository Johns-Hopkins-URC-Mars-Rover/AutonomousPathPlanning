#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>

#include <array>
#include <optional>
#include <string>
#include <utility>

#include "rover_imu_fusion/imu_transform.hpp"

namespace rover_imu_fusion
{

// Rotates sensor_msgs/Imu messages from the sensor frame into base_frame using
// TF. One instance per IMU: it caches that IMU's mounting rotation and keeps
// its own warning state. Lever-arm terms are ignored.
class ImuRotator
{
public:
  struct Options
  {
    std::string name;  // used in log messages
    std::string base_frame;
    // Diagonal variances (sensor axes) that replace the message covariance.
    // Not inflated.
    std::optional<Eigen::Vector3d> gyro_override;
    std::optional<Eigen::Vector3d> accel_override;
    // Diagonal variances used only when the incoming covariance is missing.
    std::optional<Eigen::Vector3d> gyro_fallback;
    std::optional<Eigen::Vector3d> accel_fallback;
    // Multiplier on message and fallback accel covariance, to absorb the
    // unmodelled lever-arm effect.
    double accel_inflation{1.0};
    // When true a sample with no usable covariance is dropped instead of being
    // published with zeros ("unknown").
    bool drop_without_covariance{false};
  };

  ImuRotator(rclcpp::Node & node, tf2_ros::Buffer & tf_buffer, Options options)
  : node_(node), tf_buffer_(tf_buffer), opt_(std::move(options)) {}

  // Rotation taking sensor-frame vectors to base-frame vectors. Valid once a
  // message has been converted (hasExtrinsic()).
  bool hasExtrinsic() const {return !cached_frame_.empty();}
  const Eigen::Matrix3d & rotation() const {return R_base_imu_;}

  // Returns the message expressed in base_frame, or nullopt if it cannot be
  // produced yet (no TF, or covariance required but missing).
  std::optional<sensor_msgs::msg::Imu> convert(const sensor_msgs::msg::Imu & msg)
  {
    if (!ensureExtrinsic(msg.header.frame_id)) {
      return std::nullopt;
    }

    const auto gyro_cov = outputCovariance(
      msg.angular_velocity_covariance, opt_.gyro_override, opt_.gyro_fallback, 1.0, "Gyro",
      warned_gyro_);
    const auto accel_cov = outputCovariance(
      msg.linear_acceleration_covariance, opt_.accel_override, opt_.accel_fallback,
      opt_.accel_inflation, "Accel", warned_accel_);
    if (opt_.drop_without_covariance && (!gyro_cov || !accel_cov)) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "%s: dropping samples, input covariance missing and no fallback set", opt_.name.c_str());
      return std::nullopt;
    }

    sensor_msgs::msg::Imu out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = opt_.base_frame;

    const Eigen::Vector3d w(
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    const Eigen::Vector3d a(
      msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    const Eigen::Vector3d w_b = R_base_imu_ * w;
    const Eigen::Vector3d a_b = R_base_imu_ * a;
    out.angular_velocity.x = w_b.x();
    out.angular_velocity.y = w_b.y();
    out.angular_velocity.z = w_b.z();
    out.linear_acceleration.x = a_b.x();
    out.linear_acceleration.y = a_b.y();
    out.linear_acceleration.z = a_b.z();
    out.angular_velocity_covariance = gyro_cov.value_or(std::array<double, 9>{});
    out.linear_acceleration_covariance = accel_cov.value_or(std::array<double, 9>{});

    // Orientation is forwarded only if it has a usable covariance; otherwise
    // flagged unavailable (REP-145).
    const Eigen::Quaterniond q(
      msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    const Eigen::Matrix3d q_cov = covarianceFromArray(msg.orientation_covariance);
    if (q.norm() > 0.5 && isCovarianceProvided(q_cov)) {
      const Eigen::Quaterniond qb = baseOrientation(q, R_base_imu_);
      out.orientation.w = qb.w();
      out.orientation.x = qb.x();
      out.orientation.y = qb.y();
      out.orientation.z = qb.z();
      out.orientation_covariance = msg.orientation_covariance;
    } else {
      out.orientation.w = 1.0;
      out.orientation_covariance[0] = -1.0;
    }
    return out;
  }

private:
  rclcpp::Node & node_;
  tf2_ros::Buffer & tf_buffer_;
  Options opt_;

  std::string cached_frame_;
  Eigen::Matrix3d R_base_imu_{Eigen::Matrix3d::Identity()};
  bool warned_gyro_{false};
  bool warned_accel_{false};

  // The IMU mounting is rigid, so the rotation is looked up once per frame.
  bool ensureExtrinsic(const std::string & frame)
  {
    if (frame == cached_frame_) {
      return true;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(opt_.base_frame, frame, tf2::TimePointZero);
      R_base_imu_ = tf2::transformToEigen(tf).rotation();
      cached_frame_ = frame;
      RCLCPP_INFO(
        node_.get_logger(), "%s: got TF %s <- %s", opt_.name.c_str(), opt_.base_frame.c_str(),
        frame.c_str());
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 2000, "%s: waiting for TF %s <- %s: %s",
        opt_.name.c_str(), opt_.base_frame.c_str(), frame.c_str(), e.what());
      return false;
    }
  }

  // Rotated override, message covariance, or fallback (in that order); nullopt
  // if none is available.
  std::optional<std::array<double, 9>> outputCovariance(
    const std::array<double, 9> & in, const std::optional<Eigen::Vector3d> & override_var,
    const std::optional<Eigen::Vector3d> & fallback, double inflation, const char * what,
    bool & warned)
  {
    if (override_var.has_value()) {
      return covarianceToArray(
        rotateCovariance(R_base_imu_, Eigen::Matrix3d(override_var->asDiagonal())));
    }
    const Eigen::Matrix3d cov = covarianceFromArray(in);
    if (isCovarianceProvided(cov)) {
      return covarianceToArray(rotateCovariance(R_base_imu_, inflation * cov));
    }
    if (fallback.has_value()) {
      return covarianceToArray(
        rotateCovariance(R_base_imu_, inflation * Eigen::Matrix3d(fallback->asDiagonal())));
    }
    if (!warned) {
      warned = true;
      RCLCPP_WARN(
        node_.get_logger(), "%s: %s covariance missing in input and no fallback set",
        opt_.name.c_str(), what);
    }
    return std::nullopt;
  }
};

}  // namespace rover_imu_fusion
