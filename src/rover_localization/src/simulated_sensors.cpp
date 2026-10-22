#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "simulation_model.hpp"

namespace rover_localization {

class SimulatedSensors : public rclcpp::Node {
 public:
  SimulatedSensors()
      : Node("simulated_sensors"),
        previous_time_(0, 0, get_clock()->get_clock_type()),
        start_time_(0, 0, get_clock()->get_clock_type()) {
    linear_speed_ = declare_parameter<double>("linear_speed", 0.5);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.2);
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
    position_noise_std_ = declare_parameter<double>("position_noise_std", 0.05);
    yaw_noise_std_ = declare_parameter<double>("yaw_noise_std", 0.02);
    gyro_noise_std_ = declare_parameter<double>("gyro_noise_std", 0.01);
    camera_dropout_start_ = declare_parameter<double>("camera_dropout_start", -1.0);
    camera_dropout_duration_ = declare_parameter<double>("camera_dropout_duration", 0.0);
    imu_dropout_start_ = declare_parameter<double>("imu_dropout_start", -1.0);
    imu_dropout_duration_ = declare_parameter<double>("imu_dropout_duration", 0.0);
    const auto odom_topic = declare_parameter<std::string>(
        "zed_odom_topic", "/zed/zed_node/odom");
    const auto imu_topic = declare_parameter<std::string>("fused_imu_topic", "/imu/fused");

    if (!std::isfinite(linear_speed_) || !std::isfinite(angular_speed_) ||
        !std::isfinite(publish_rate_) || publish_rate_ <= 0.0 ||
        !std::isfinite(position_noise_std_) || position_noise_std_ < 0.0 ||
        !std::isfinite(yaw_noise_std_) || yaw_noise_std_ < 0.0 ||
        !std::isfinite(gyro_noise_std_) || gyro_noise_std_ < 0.0 ||
        !std::isfinite(camera_dropout_duration_) || camera_dropout_duration_ < 0.0 ||
        !std::isfinite(imu_dropout_duration_) || imu_dropout_duration_ < 0.0) {
      throw std::invalid_argument("Invalid speed, rate, noise, or dropout duration");
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS());
    truth_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        "/simulation/ground_truth", 10);

    start_time_ = now();
    previous_time_ = start_time_;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_));
    if (period.count() <= 0) {
      throw std::invalid_argument("publish_rate is too high");
    }
    timer_ = create_wall_timer(period, std::bind(&SimulatedSensors::update, this));
    RCLCPP_INFO(get_logger(), "Synthetic optical odom: %s; joint IMU: %s",
                odom_topic.c_str(), imu_topic.c_str());
  }

 private:
  bool in_dropout(double elapsed, double start, double duration) const {
    return start >= 0.0 && elapsed >= start && elapsed < start + duration;
  }

  double sample_noise(double stddev) {
    return stddev == 0.0 ? 0.0 : normal_(rng_) * stddev;
  }

  void update() {
    const auto stamp = now();
    const double dt = (stamp - previous_time_).seconds();
    previous_time_ = stamp;
    if (dt <= 0.0) {
      return;
    }
    pose_ = advance_pose(pose_, linear_speed_, angular_speed_, dt);
    const double elapsed = (stamp - start_time_).seconds();
    publish_truth(stamp);
    if (!in_dropout(elapsed, camera_dropout_start_, camera_dropout_duration_)) {
      publish_optical_odom(stamp);
    }
    if (!in_dropout(elapsed, imu_dropout_start_, imu_dropout_duration_)) {
      publish_joint_imu(stamp);
    }
  }

  void publish_truth(const rclcpp::Time &stamp) {
    nav_msgs::msg::Odometry truth;
    truth.header.stamp = stamp;
    truth.header.frame_id = "odom";
    truth.child_frame_id = "base_link";
    truth.pose.pose.position.x = pose_.x;
    truth.pose.pose.position.y = pose_.y;
    truth.pose.pose.orientation.z = std::sin(pose_.yaw / 2.0);
    truth.pose.pose.orientation.w = std::cos(pose_.yaw / 2.0);
    truth.twist.twist.linear.x = linear_speed_;
    truth.twist.twist.angular.z = angular_speed_;
    truth_publisher_->publish(truth);
  }

  void publish_optical_odom(const rclcpp::Time &stamp) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    // In this synthetic rig the virtual camera is mounted at base_link.
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pose_.x + sample_noise(position_noise_std_);
    odom.pose.pose.position.y = pose_.y + sample_noise(position_noise_std_);
    const double observed_yaw = wrap_angle(pose_.yaw + sample_noise(yaw_noise_std_));
    odom.pose.pose.orientation.z = std::sin(observed_yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(observed_yaw / 2.0);
    odom.pose.covariance[0] = position_noise_std_ * position_noise_std_ + 1e-4;
    odom.pose.covariance[7] = odom.pose.covariance[0];
    odom.pose.covariance[14] = 0.01;
    odom.pose.covariance[21] = 0.01;
    odom.pose.covariance[28] = 0.01;
    odom.pose.covariance[35] = yaw_noise_std_ * yaw_noise_std_ + 1e-4;
    odom_publisher_->publish(odom);
  }

  void publish_joint_imu(const rclcpp::Time &stamp) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_link";
    imu.angular_velocity.z = angular_speed_ + sample_noise(gyro_noise_std_);
    imu.angular_velocity_covariance[0] = 0.001;
    imu.angular_velocity_covariance[4] = 0.001;
    imu.angular_velocity_covariance[8] = gyro_noise_std_ * gyro_noise_std_ + 1e-5;
    imu.orientation_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_publisher_->publish(imu);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time previous_time_;
  rclcpp::Time start_time_;
  PlanarPose pose_{0.0, 0.0, 0.0};
  std::mt19937 rng_{7};
  std::normal_distribution<double> normal_{0.0, 1.0};
  double linear_speed_;
  double angular_speed_;
  double publish_rate_;
  double position_noise_std_;
  double yaw_noise_std_;
  double gyro_noise_std_;
  double camera_dropout_start_;
  double camera_dropout_duration_;
  double imu_dropout_start_;
  double imu_dropout_duration_;
};

}  // namespace rover_localization

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_localization::SimulatedSensors>());
  rclcpp::shutdown();
  return 0;
}
