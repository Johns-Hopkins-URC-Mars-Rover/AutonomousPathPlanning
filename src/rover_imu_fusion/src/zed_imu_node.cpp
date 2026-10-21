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

#include "rover_imu_fusion/imu_rotator.hpp"

namespace rover_imu_fusion
{

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
