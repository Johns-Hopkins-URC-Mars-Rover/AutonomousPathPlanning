#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <memory>
#include <stdexcept>


namespace rover_localization {
  class SimulatedSensors : public rclcpp::Node {
  public:
    SimulatedSensors(): Node("simulated_sensors"), previous_time(0, 0, this->get_clock()->get_clock_type()) {
      linear_speed = declare_parameter<double>("linear_speed", 0.5);
      angular_speed = declare_parameter<double>("angular_speed", 0.2);
      publish_rate = declare_parameter<double>("publish_rate", 20.0);

      if (!std::isfinite(publish_rate) || publish_rate <= 0.0 || !std::isfinite(linear_speed) || !std::isfinite(angular_speed)) {
        throw std::invalid_argument("Speeds must be finite and publish rate must be positive");
      }

      odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 10);
      imu_publisher = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
      ground_truth_publisher = create_publisher<nav_msgs::msg::Odometry>("/simulation/ground_truth", 10);

      previous_time = now();
      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate));
      if (period.count() <= 0) {
        throw std::invalid_argument("Publish rate is too high");
      }

      timer = create_wall_timer(period, std::bind(&SimulatedSensors::Update, this));

      RCLCPP_INFO(get_logger(), "Simulating foward speed %.2f m/s and turn rate %.2f rad/s", linear_speed, angular_speed);
    }

  private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_publisher;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time previous_time;

    double x{0.0};
    double y{0.0};
    double yaw{0.0};

    double linear_speed;
    double angular_speed;
    double publish_rate;

    void Update() {
      const auto current_time = now();
      const double dt = (current_time - previous_time).seconds();
      previous_time = current_time;

      if (dt <= 0.0) {
	      return;
      }

      AdvanceState(dt);
      PublishMeasurements(current_time);
    }

    void AdvanceState(double dt) {
      const double next_yaw = yaw + angular_speed * dt;

      if (std::abs(angular_speed) < 1e-6) {
        x += linear_speed * std::cos(yaw) * dt;
        y += linear_speed * std::sin(yaw) * dt;
      }
      else {
        const double radius = linear_speed / angular_speed;
        x += radius * (std::sin(next_yaw) - std::sin(yaw));
        y += radius * (std::cos(yaw) - std::cos(next_yaw));
      }
      yaw = std::atan2(std::sin(next_yaw), std::cos(next_yaw));
    }

    void PublishMeasurements(const rclcpp::Time &stamp) {
      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp = stamp;
      odometry.header.frame_id = "odom";
      odometry.child_frame_id = "base_link";

      // Wheel odometry supplies forward velocity in the rover's frame.
      odometry.twist.twist.linear.x = linear_speed;
      odometry.twist.twist.linear.y = 0.0;
      odometry.twist.covariance[0] = 0.01;

      // Other twist components are not measured by this publisher.
      for (const int index : {7, 14, 21, 28, 35})
      {
          odometry.twist.covariance[index] = 1e6;
      }

      // This message does not supply a usable pose measurement.
      odometry.pose.pose.orientation.w = 1.0;
      odometry.twist.covariance[7] = 0.01;

      for (const int index : {0, 7, 14, 21, 28, 35})
      {
          odometry.pose.covariance[index] = 1e6;
      }

      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;

      // For this simulation, the IMU is aligned with the rover's frame.
      imu.header.frame_id = "base_link";

      imu.angular_velocity.z = angular_speed;
      imu.angular_velocity_covariance[0] = 0.001;
      imu.angular_velocity_covariance[4] = 0.001;
      imu.angular_velocity_covariance[8] = 0.001;

      // We are not simulating orientation or acceleration measurements.
      imu.orientation_covariance[0] = -1.0;
      imu.linear_acceleration_covariance[0] = -1.0;

      nav_msgs::msg::Odometry ground_truth;
      ground_truth.header.stamp = stamp;
      ground_truth.header.frame_id = "odom";
      ground_truth.child_frame_id = "base_link";

      ground_truth.pose.pose.position.x = x;
      ground_truth.pose.pose.position.y = y;

      // Represent the planar heading as a quaternion.
      ground_truth.pose.pose.orientation.z = std::sin(yaw / 2.0);
      ground_truth.pose.pose.orientation.w = std::cos(yaw / 2.0);

      ground_truth.twist.twist.linear.x = linear_speed;
      ground_truth.twist.twist.angular.z = angular_speed;

      odometry_publisher->publish(odometry);
      imu_publisher->publish(imu);
      ground_truth_publisher->publish(ground_truth);
    }
  };
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rover_localization::SimulatedSensors>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
      
