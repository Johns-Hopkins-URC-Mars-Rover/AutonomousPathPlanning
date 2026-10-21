#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>


namespace rover_localization {
  class SimulatedSensors : public rclcpp::Node {
  public:
    SimulatedSensors(): Node("simulated_sensors"), previous_time(0, 0, this->get_clock()->get_clock_type()) {
      linear_speed = declare_parameter<double>("linear_speed", 0.5);
      angular_speed = declare_parameter<double>("angular_speed", 0.2);
      publish_rate = declare_parameter<double>("publish_rate", 20.0);
      // The rover stands still for this long before driving, so a filter that
      // needs a stationary start can initialize.
      start_delay = declare_parameter<double>("start_delay", 0.0);

      if (!std::isfinite(publish_rate) || publish_rate <= 0.0 || !std::isfinite(linear_speed) || !std::isfinite(angular_speed)) {
        throw std::invalid_argument("Speeds must be finite and publish rate must be positive");
      }

      odometry_publisher = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 10);
      // The simulated IMU is kept off /imu/data so that it cannot collide with
      // the output of rover_imu_fusion.
      const auto imu_topic = declare_parameter<std::string>("imu_topic", "/sim/imu");
      imu_publisher = create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
      ground_truth_publisher = create_publisher<nav_msgs::msg::Odometry>("/simulation/ground_truth", 10);

      previous_time = now();
      start_time = now();
      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate));
      if (period.count() <= 0) {
        throw std::invalid_argument("Publish rate is too high");
      }

      timer = create_wall_timer(period, std::bind(&SimulatedSensors::Update, this));

      SetupSyntheticImus();

      RCLCPP_INFO(get_logger(), "Simulating foward speed %.2f m/s and turn rate %.2f rad/s", linear_speed, angular_speed);
    }

  private:
    // Optional synthetic IMUs on the hardware topic names (/imu/data_raw and
    // /zed/zed_node/imu/data) with known bias and noise, so that rover_imu_fusion
    // can be tested without hardware. Both are aligned with base_link, so no
    // extra TF is needed. The rover drives a constant circle, so the true body
    // rate is (0, 0, w) and the true specific force is (0, v w, g).
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr icm_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr zed_publisher;
    rclcpp::TimerBase::SharedPtr synthetic_timer;
    std::mt19937 rng{0};
    std::normal_distribution<double> unit_normal{0.0, 1.0};
    double gyro_noise_std{0.01};
    double accel_noise_std{0.05};
    double zed_gyro_noise_std{0.005};
    double zed_accel_noise_std{0.02};
    std::vector<double> icm_gyro_bias;
    std::vector<double> icm_accel_bias;

    static std::vector<double> CheckedVector3(std::vector<double> v, const char * name) {
      if (v.size() != 3) {
        throw std::invalid_argument(std::string(name) + " must have 3 values");
      }
      return v;
    }

    void SetupSyntheticImus() {
      if (!declare_parameter<bool>("publish_synthetic_imus", false)) {
        return;
      }
      const double rate = declare_parameter<double>("synthetic_imu_rate", 100.0);
      if (!std::isfinite(rate) || rate <= 0.0) {
        throw std::invalid_argument("synthetic_imu_rate must be positive");
      }
      rng.seed(static_cast<unsigned>(declare_parameter<int64_t>("synthetic_seed", 0)));
      gyro_noise_std = declare_parameter<double>("icm_gyro_noise_std", 0.01);
      accel_noise_std = declare_parameter<double>("icm_accel_noise_std", 0.05);
      zed_gyro_noise_std = declare_parameter<double>("zed_gyro_noise_std", 0.005);
      zed_accel_noise_std = declare_parameter<double>("zed_accel_noise_std", 0.02);
      icm_gyro_bias = CheckedVector3(
        declare_parameter<std::vector<double>>("icm_gyro_bias", {0.02, -0.01, 0.015}), "icm_gyro_bias");
      icm_accel_bias = CheckedVector3(
        declare_parameter<std::vector<double>>("icm_accel_bias", {0.05, 0.0, -0.03}), "icm_accel_bias");

      icm_publisher = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS());
      zed_publisher = create_publisher<sensor_msgs::msg::Imu>(
        "/zed/zed_node/imu/data", rclcpp::SensorDataQoS());
      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate));
      synthetic_timer = create_wall_timer(period, std::bind(&SimulatedSensors::PublishSyntheticImus, this));
      RCLCPP_INFO(get_logger(), "Publishing synthetic IMUs at %.0f Hz (ICM bias gyro [%.3f %.3f %.3f] accel [%.3f %.3f %.3f])",
        rate, icm_gyro_bias[0], icm_gyro_bias[1], icm_gyro_bias[2], icm_accel_bias[0], icm_accel_bias[1], icm_accel_bias[2]);
    }

    sensor_msgs::msg::Imu MakeSyntheticImu(
      const rclcpp::Time & stamp, const std::vector<double> & gyro_bias, const std::vector<double> & accel_bias,
      double gyro_std, double accel_std) {
      constexpr double gravity = 9.80665;
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = "base_link";

      imu.angular_velocity.x = gyro_bias[0] + gyro_std * unit_normal(rng);
      imu.angular_velocity.y = gyro_bias[1] + gyro_std * unit_normal(rng);
      imu.angular_velocity.z = w_now + gyro_bias[2] + gyro_std * unit_normal(rng);
      imu.linear_acceleration.x = accel_bias[0] + accel_std * unit_normal(rng);
      imu.linear_acceleration.y = v_now * w_now + accel_bias[1] + accel_std * unit_normal(rng);
      imu.linear_acceleration.z = gravity + accel_bias[2] + accel_std * unit_normal(rng);

      for (const int index : {0, 4, 8}) {
        imu.angular_velocity_covariance[index] = gyro_std * gyro_std;
        imu.linear_acceleration_covariance[index] = accel_std * accel_std;
      }
      imu.orientation_covariance[0] = -1.0;
      return imu;
    }

    void PublishSyntheticImus() {
      UpdateEffectiveSpeeds();
      const auto stamp = now();
      const std::vector<double> zero(3, 0.0);
      // ICM: biased, noisier. ZED: SDK-corrected, so no bias.
      icm_publisher->publish(MakeSyntheticImu(stamp, icm_gyro_bias, icm_accel_bias, gyro_noise_std, accel_noise_std));
      zed_publisher->publish(MakeSyntheticImu(stamp, zero, zero, zed_gyro_noise_std, zed_accel_noise_std));
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_publisher;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time previous_time;
    rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
    double start_delay{0.0};
    // Speeds actually applied now (zero until start_delay has passed).
    double v_now{0.0};
    double w_now{0.0};

    double x{0.0};
    double y{0.0};
    double yaw{0.0};

    double linear_speed;
    double angular_speed;
    double publish_rate;

    void UpdateEffectiveSpeeds() {
      const bool moving = (now() - start_time).seconds() >= start_delay;
      v_now = moving ? linear_speed : 0.0;
      w_now = moving ? angular_speed : 0.0;
    }

    void Update() {
      UpdateEffectiveSpeeds();
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
      const double next_yaw = yaw + w_now * dt;

      if (std::abs(w_now) < 1e-6) {
        x += v_now * std::cos(yaw) * dt;
        y += v_now * std::sin(yaw) * dt;
      }
      else {
        const double radius = v_now / w_now;
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
      odometry.twist.twist.linear.x = v_now;
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

      imu.angular_velocity.z = w_now;
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

      ground_truth.twist.twist.linear.x = v_now;
      ground_truth.twist.twist.angular.z = w_now;

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
      
