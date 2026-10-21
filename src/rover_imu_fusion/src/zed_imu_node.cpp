#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rover_imu_fusion/imu_transform.hpp"

namespace rover_imu_fusion
{

// Takes the ZED IMU message, rotates it into base_frame using TF, and
// republishes it. No filtering: values are the ZED SDK's own.
class ZedImuNode : public rclcpp::Node
{
public:
  ZedImuNode()
  : Node("zed_imu"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/zed/zed_node/imu/data");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/imu/data");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // Optional diagonal variances used only when the incoming covariance is
    // missing. Unset by default: no invented numbers.
    gyro_fallback_ = declareFallback("angular_velocity_variance");
    accel_fallback_ = declareFallback("linear_acceleration_variance");

    // The ZED publishes this topic only while someone subscribes to it.
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&ZedImuNode::onImu, this, std::placeholders::_1));
    // Reliable so that both reliable and best-effort subscribers can connect.
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);

    RCLCPP_INFO(
      get_logger(), "Forwarding %s -> %s in frame %s", input_topic.c_str(),
      output_topic.c_str(), base_frame_.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string base_frame_;
  std::string cached_frame_;
  Eigen::Matrix3d R_base_imu_{Eigen::Matrix3d::Identity()};

  std::optional<Eigen::Vector3d> gyro_fallback_;
  std::optional<Eigen::Vector3d> accel_fallback_;
  bool warned_gyro_{false};
  bool warned_accel_{false};
  bool warned_orientation_{false};

  std::optional<Eigen::Vector3d> declareFallback(const std::string & name)
  {
    const auto v = declare_parameter<std::vector<double>>(name, std::vector<double>{});
    if (v.empty()) {
      return std::nullopt;
    }
    if (v.size() != 3 || v[0] <= 0.0 || v[1] <= 0.0 || v[2] <= 0.0) {
      throw std::invalid_argument(name + " must be empty or 3 positive values");
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  // The IMU mounting is rigid, so the rotation is looked up once per frame.
  bool ensureExtrinsic(const std::string & frame)
  {
    if (frame == cached_frame_) {
      return true;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, frame, tf2::TimePointZero);
      R_base_imu_ = tf2::transformToEigen(tf).rotation();
      cached_frame_ = frame;
      RCLCPP_INFO(get_logger(), "Got TF %s <- %s", base_frame_.c_str(), frame.c_str());
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for TF %s <- %s: %s",
        base_frame_.c_str(), frame.c_str(), e.what());
      return false;
    }
  }

  // Rotated covariance, a diagonal fallback, or zeros ("unknown").
  std::array<double, 9> outputCovariance(
    const std::array<double, 9> & in, const std::optional<Eigen::Vector3d> & fallback,
    bool & warned, const char * what)
  {
    const Eigen::Matrix3d cov = covarianceFromArray(in);
    if (isCovarianceProvided(cov)) {
      return covarianceToArray(rotateCovariance(R_base_imu_, cov));
    }
    if (fallback.has_value()) {
      return covarianceToArray(
        rotateCovariance(R_base_imu_, Eigen::Matrix3d(fallback->asDiagonal())));
    }
    if (!warned) {
      warned = true;
      RCLCPP_WARN(
        get_logger(), "%s covariance missing in input and no fallback set; publishing zeros", what);
    }
    return std::array<double, 9>{};
  }

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (!ensureExtrinsic(msg->header.frame_id)) {
      return;
    }

    sensor_msgs::msg::Imu out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = base_frame_;

    const Eigen::Vector3d w(
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    const Eigen::Vector3d a(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    const Eigen::Vector3d w_b = R_base_imu_ * w;
    const Eigen::Vector3d a_b = R_base_imu_ * a;
    out.angular_velocity.x = w_b.x();
    out.angular_velocity.y = w_b.y();
    out.angular_velocity.z = w_b.z();
    out.linear_acceleration.x = a_b.x();
    out.linear_acceleration.y = a_b.y();
    out.linear_acceleration.z = a_b.z();

    out.angular_velocity_covariance =
      outputCovariance(msg->angular_velocity_covariance, gyro_fallback_, warned_gyro_, "Gyro");
    out.linear_acceleration_covariance =
      outputCovariance(
      msg->linear_acceleration_covariance, accel_fallback_, warned_accel_,
      "Accel");

    // Orientation is the ZED's own estimate, forwarded only if it has a
    // usable covariance; otherwise flagged unavailable (REP-145).
    const Eigen::Quaterniond q(
      msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    const Eigen::Matrix3d q_cov = covarianceFromArray(msg->orientation_covariance);
    if (q.norm() > 0.5 && isCovarianceProvided(q_cov)) {
      const Eigen::Quaterniond qb = baseOrientation(q, R_base_imu_);
      out.orientation.w = qb.w();
      out.orientation.x = qb.x();
      out.orientation.y = qb.y();
      out.orientation.z = qb.z();
      out.orientation_covariance = msg->orientation_covariance;
    } else {
      out.orientation.w = 1.0;
      out.orientation_covariance[0] = -1.0;
      if (!warned_orientation_) {
        warned_orientation_ = true;
        RCLCPP_WARN(get_logger(), "Input orientation unusable; publishing it as unavailable");
      }
    }

    publisher_->publish(out);
  }
};

}  // namespace rover_imu_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_imu_fusion::ZedImuNode>());
  rclcpp::shutdown();
  return 0;
}
