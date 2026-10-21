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
#include <utility>
#include <vector>

#include "rover_imu_fusion/imu_transform.hpp"

namespace rover_imu_fusion
{

// Rotates sensor_msgs/Imu messages from the sensor frame into base_frame using
// TF. One instance per IMU: it caches that IMU's mounting rotation and keeps
// its own warning state.
class ImuRotator
{
public:
  struct Options
  {
    std::string name;  // used in log messages
    std::string base_frame;
    // Diagonal variances used only when the incoming covariance is missing.
    std::optional<Eigen::Vector3d> gyro_fallback;
    std::optional<Eigen::Vector3d> accel_fallback;
    // When true a sample with a missing covariance is dropped instead of being
    // published with zeros ("unknown").
    bool drop_without_covariance{false};
  };

  ImuRotator(rclcpp::Node & node, tf2_ros::Buffer & tf_buffer, Options options)
  : node_(node), tf_buffer_(tf_buffer), opt_(std::move(options)) {}

  // Returns the message expressed in base_frame, or nullopt if it cannot be
  // produced yet (no TF, or covariance required but missing).
  std::optional<sensor_msgs::msg::Imu> convert(const sensor_msgs::msg::Imu & msg)
  {
    if (!ensureExtrinsic(msg.header.frame_id)) {
      return std::nullopt;
    }

    const auto gyro_cov = outputCovariance(
      msg.angular_velocity_covariance, opt_.gyro_fallback, "Gyro", warned_gyro_);
    const auto accel_cov = outputCovariance(
      msg.linear_acceleration_covariance, opt_.accel_fallback, "Accel", warned_accel_);
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

  // Rotated covariance, a rotated diagonal fallback, or nullopt if neither.
  std::optional<std::array<double, 9>> outputCovariance(
    const std::array<double, 9> & in, const std::optional<Eigen::Vector3d> & fallback,
    const char * what, bool & warned)
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
        node_.get_logger(), "%s: %s covariance missing in input and no fallback set",
        opt_.name.c_str(), what);
    }
    return std::nullopt;
  }
};

// ZED path: rotates the ZED IMU into base_frame and republishes it as
// /imu/data. No filtering: values are the ZED SDK's own.
//
// ICM-20948 path (off by default, icm.enabled): rotates the ICM IMU into
// base_frame and publishes it on ~/icm_base so the mount transform and the
// covariances can be checked. Samples without a covariance are dropped. It is
// not fused yet; enable it once the driver publishes measured covariances.
class ZedImuNode : public rclcpp::Node
{
public:
  ZedImuNode()
  : Node("zed_imu"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/zed/zed_node/imu/data");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/imu/data");
    const auto base_frame = declare_parameter<std::string>("base_frame", "base_link");

    // Optional diagonal variances used only when the incoming covariance is
    // missing. Unset by default: no invented numbers.
    ImuRotator::Options zed_opt;
    zed_opt.name = "zed";
    zed_opt.base_frame = base_frame;
    zed_opt.gyro_fallback = declareFallback("angular_velocity_variance");
    zed_opt.accel_fallback = declareFallback("linear_acceleration_variance");
    zed_rotator_ = std::make_unique<ImuRotator>(*this, tf_buffer_, zed_opt);

    // The ZED publishes this topic only while someone subscribes to it.
    zed_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        if (const auto out = zed_rotator_->convert(*msg)) {
          publisher_->publish(*out);
        }
      });
    // Reliable so that both reliable and best-effort subscribers can connect.
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);
    RCLCPP_INFO(
      get_logger(), "Forwarding %s -> %s in frame %s", input_topic.c_str(),
      output_topic.c_str(), base_frame.c_str());

    // ICM-20948. Needs a measured TF base_link -> imu_link and real
    // covariances from the driver.
    const bool icm_enabled = declare_parameter<bool>("icm.enabled", false);
    const auto icm_topic = declare_parameter<std::string>("icm.input_topic", "/imu/data_raw");
    if (icm_enabled) {
      ImuRotator::Options icm_opt;
      icm_opt.name = "icm";
      icm_opt.base_frame = base_frame;
      icm_opt.drop_without_covariance = true;
      icm_rotator_ = std::make_unique<ImuRotator>(*this, tf_buffer_, icm_opt);

      icm_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        icm_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          if (const auto out = icm_rotator_->convert(*msg)) {
            icm_publisher_->publish(*out);
          }
        });
      icm_publisher_ = create_publisher<sensor_msgs::msg::Imu>("~/icm_base", 10);
      RCLCPP_INFO(get_logger(), "ICM enabled: %s -> ~/icm_base", icm_topic.c_str());
    }
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<ImuRotator> zed_rotator_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr zed_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

  std::unique_ptr<ImuRotator> icm_rotator_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr icm_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr icm_publisher_;

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
};

}  // namespace rover_imu_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_imu_fusion::ZedImuNode>());
  rclcpp::shutdown();
  return 0;
}
