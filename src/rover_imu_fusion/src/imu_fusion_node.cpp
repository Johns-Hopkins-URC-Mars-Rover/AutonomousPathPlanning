#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rover_imu_fusion/imu_fusion.hpp"
#include "rover_imu_fusion/imu_rotator.hpp"

namespace rover_imu_fusion
{

namespace
{

constexpr int kIcm = 0;
constexpr int kZed = 1;

double toSeconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

builtin_interfaces::msg::Time toStamp(double seconds)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(seconds));
  t.nanosec = static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9);
  return t;
}

}  // namespace

// Fuses the ICM-20948 and ZED IMUs into one sensor_msgs/Imu on /imu/data.
// All the filter logic lives in the ROS-free ImuFusion class.
class ImuFusionNode : public rclcpp::Node
{
public:
  ImuFusionNode()
  : Node("imu_fusion"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/imu/data");
    const auto diag_topic = declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    const double publish_rate = declare_parameter<double>("publish_rate", 100.0);
    if (!(publish_rate > 0.0)) {
      throw std::invalid_argument("publish_rate must be positive");
    }

    FusionConfig cfg = declareFusionConfig();
    reorder_delay_ = cfg.reorder_delay;
    fusion_ = std::make_unique<ImuFusion>(cfg);

    setupImu(kIcm, "icm", "/imu/data_raw", false);
    setupImu(kZed, "zed", "/zed/zed_node/imu/data", true);
    setupMagnetometer();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);
    diag_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diag_topic, 10);
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate), [this]() {onPublishTimer();});
    diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {onDiagnosticsTimer();});

    RCLCPP_INFO(
      get_logger(), "IMU fusion publishing %s in %s at %.0f Hz (waiting for a stationary start)",
      output_topic.c_str(), base_frame_.c_str(), publish_rate);
  }

private:
  struct Input
  {
    bool enabled{false};
    std::string name;
    std::string topic;
    std::unique_ptr<ImuRotator> rotator;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription;
  };

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_frame_;
  double reorder_delay_{0.015};
  std::unique_ptr<ImuFusion> fusion_;
  std::array<Input, 2> inputs_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Magnetometer (optional).
  bool use_mag_{false};
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
  Eigen::Vector3d mag_hard_iron_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d mag_soft_iron_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d mag_to_imu_{Eigen::Matrix3d::Identity()};

  std::vector<double> param_vector(const std::string & name, const std::vector<double> & def)
  {
    return declare_parameter<std::vector<double>>(name, def);
  }

  std::optional<Eigen::Vector3d> optionalVariance(const std::string & name)
  {
    const auto v = param_vector(name, {});
    if (v.empty()) {
      return std::nullopt;
    }
    if (v.size() != 3 || v[0] <= 0.0 || v[1] <= 0.0 || v[2] <= 0.0) {
      throw std::invalid_argument(name + " must be empty or 3 positive values");
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  Eigen::Matrix3d matrixParam(const std::string & name, const Eigen::Matrix3d & def)
  {
    const auto v = param_vector(
      name, {def(0, 0), def(0, 1), def(0, 2), def(1, 0), def(1, 1),
        def(1, 2), def(2, 0), def(2, 1), def(2, 2)});
    if (v.size() != 9) {
      throw std::invalid_argument(name + " must have 9 values (row-major)");
    }
    Eigen::Matrix3d m;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        m(r, c) = v[static_cast<std::size_t>(r * 3 + c)];
      }
    }
    return m;
  }

  // Filter tuning. These are starting points for the filter, not measured
  // sensor properties; sensor noise comes from the IMU messages or overrides.
  FusionConfig declareFusionConfig()
  {
    FusionConfig c;
    c.reorder_delay = declare_parameter<double>("reorder_delay", 0.015);
    c.imu_timeout = declare_parameter<double>("imu_timeout", 0.5);
    c.gravity_update_period = declare_parameter<double>("gravity_update_period", 0.1);

    auto & e = c.ekf;
    e.rate_walk = declare_parameter<double>("filter.rate_walk", 1.0);
    e.accel_walk = declare_parameter<double>("filter.accel_walk", 5.0);
    e.init_attitude_sigma = declare_parameter<double>("filter.init_attitude_sigma", 0.05);
    e.init_rate_sigma = declare_parameter<double>("filter.init_rate_sigma", 0.05);
    e.init_accel_sigma = declare_parameter<double>("filter.init_accel_sigma", 0.5);
    e.gravity = declare_parameter<double>("filter.gravity", 9.80665);
    e.gravity_sigma = declare_parameter<double>("filter.gravity_sigma", 0.5);
    e.gravity_adapt = declare_parameter<double>("filter.gravity_adapt", 0.5);
    e.gravity_skip = declare_parameter<double>("filter.gravity_skip", 2.0);
    e.mag_yaw_sigma = declare_parameter<double>("filter.mag_yaw_sigma", 0.1);
    e.nis_gate_3dof = declare_parameter<double>("filter.nis_gate_3dof", 16.27);
    e.nis_gate_1dof = declare_parameter<double>("filter.nis_gate_1dof", 10.83);
    e.nis_max_consecutive =
      static_cast<int>(declare_parameter<int64_t>("filter.nis_max_consecutive", 20));
    e.stuck_count = static_cast<int>(declare_parameter<int64_t>("filter.stuck_count", 20));

    // Per IMU.
    e.gyro_bias_walk[kIcm] = declare_parameter<double>("icm.gyro_bias_walk", 1e-4);
    e.accel_bias_walk[kIcm] = declare_parameter<double>("icm.accel_bias_walk", 1e-3);
    e.init_gyro_bias_sigma[kIcm] = declare_parameter<double>("icm.init_gyro_bias_sigma", 0.01);
    e.init_accel_bias_sigma[kIcm] = declare_parameter<double>("icm.init_accel_bias_sigma", 0.1);
    e.gyro_saturation[kIcm] = declare_parameter<double>("icm.gyro_saturation", 33.0);
    e.accel_saturation[kIcm] = declare_parameter<double>("icm.accel_saturation", 149.0);
    e.gyro_bias_walk[kZed] = declare_parameter<double>("zed.gyro_bias_walk", 1e-5);
    e.accel_bias_walk[kZed] = declare_parameter<double>("zed.accel_bias_walk", 1e-4);
    e.init_gyro_bias_sigma[kZed] = declare_parameter<double>("zed.init_gyro_bias_sigma", 0.002);
    e.init_accel_bias_sigma[kZed] = declare_parameter<double>("zed.init_accel_bias_sigma", 0.05);
    e.gyro_saturation[kZed] = declare_parameter<double>("zed.gyro_saturation", 0.0);
    e.accel_saturation[kZed] = declare_parameter<double>("zed.accel_saturation", 0.0);

    auto & i = c.init;
    i.duration = declare_parameter<double>("init.duration", 1.0);
    i.min_samples = static_cast<std::size_t>(declare_parameter<int64_t>("init.min_samples", 20));
    i.gyro_std_max = declare_parameter<double>("init.gyro_std_max", 0.05);
    i.accel_std_max = declare_parameter<double>("init.accel_std_max", 0.5);
    i.gyro_mean_max = declare_parameter<double>("init.gyro_mean_max", 0.1);
    i.accel_norm_tol = declare_parameter<double>("init.accel_norm_tol", 1.0);
    i.gravity = e.gravity;
    return c;
  }

  void setupImu(int index, const std::string & name, const std::string & default_topic, bool zed)
  {
    Input & in = inputs_[static_cast<std::size_t>(index)];
    in.name = name;
    in.enabled = declare_parameter<bool>(name + ".enabled", zed);
    in.topic = declare_parameter<std::string>(name + ".topic", default_topic);

    ImuRotator::Options opt;
    opt.name = name;
    opt.base_frame = base_frame_;
    opt.gyro_override = optionalVariance(name + ".gyro_variance_override");
    opt.accel_override = optionalVariance(name + ".accel_variance_override");
    opt.gyro_fallback = optionalVariance(name + ".gyro_variance_fallback");
    opt.accel_fallback = optionalVariance(name + ".accel_variance_fallback");
    opt.accel_inflation = declare_parameter<double>(name + ".accel_inflation", 1.5);
    opt.drop_without_covariance = true;

    if (zed) {
      // Bias-corrected SDK data by default; raw needs publish_imu_raw: true on
      // the ZED (see config/zed_overrides.yaml).
      const bool use_raw = declare_parameter<bool>("zed.use_raw", false);
      const auto raw_topic =
        declare_parameter<std::string>("zed.raw_topic", "/zed/zed_node/imu/data_raw");
      if (use_raw) {
        in.topic = raw_topic;
      }
    }
    if (!in.enabled) {
      return;
    }
    in.rotator = std::make_unique<ImuRotator>(*this, tf_buffer_, opt);
    // The ZED publishes its IMU topic only while someone subscribes to it.
    in.subscription = create_subscription<sensor_msgs::msg::Imu>(
      in.topic, rclcpp::SensorDataQoS(),
      [this, index](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {onImu(index, *msg);});
    RCLCPP_INFO(get_logger(), "%s input: %s", name.c_str(), in.topic.c_str());
  }

  void setupMagnetometer()
  {
    use_mag_ = declare_parameter<bool>("use_magnetometer", false);
    const auto topic = declare_parameter<std::string>("mag.topic", "/imu/mag");
    const auto hard = param_vector("mag.hard_iron", {0.0, 0.0, 0.0});
    if (hard.size() != 3) {
      throw std::invalid_argument("mag.hard_iron must have 3 values");
    }
    mag_hard_iron_ = Eigen::Vector3d(hard[0], hard[1], hard[2]);
    mag_soft_iron_ = matrixParam("mag.soft_iron", Eigen::Matrix3d::Identity());
    mag_to_imu_ = matrixParam("mag.to_imu_rotation", Eigen::Matrix3d::Identity());
    if (!use_mag_) {
      return;
    }
    if (!inputs_[kIcm].enabled) {
      RCLCPP_WARN(
        get_logger(), "use_magnetometer needs icm.enabled (mount TF); magnetometer ignored");
      use_mag_ = false;
      return;
    }
    mag_subscription_ = create_subscription<sensor_msgs::msg::MagneticField>(
      topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {onMag(*msg);});
    RCLCPP_INFO(get_logger(), "magnetometer yaw aid enabled: %s", topic.c_str());
  }

  double stampOrNow(const builtin_interfaces::msg::Time & t)
  {
    const double s = toSeconds(t);
    return s > 0.0 ? s : now().seconds();
  }

  void onImu(int index, const sensor_msgs::msg::Imu & msg)
  {
    Input & in = inputs_[static_cast<std::size_t>(index)];
    const auto out = in.rotator->convert(msg);
    if (!out) {
      return;
    }
    const double stamp = stampOrNow(out->header.stamp);

    Measurement gyro;
    gyro.kind = Measurement::Kind::kGyro;
    gyro.imu = index;
    gyro.stamp = stamp;
    gyro.z = Eigen::Vector3d(
      out->angular_velocity.x, out->angular_velocity.y, out->angular_velocity.z);
    gyro.R = covarianceFromArray(out->angular_velocity_covariance);
    fusion_->add(gyro);

    Measurement accel;
    accel.kind = Measurement::Kind::kAccel;
    accel.imu = index;
    accel.stamp = stamp;
    accel.z = Eigen::Vector3d(
      out->linear_acceleration.x, out->linear_acceleration.y, out->linear_acceleration.z);
    accel.R = covarianceFromArray(out->linear_acceleration_covariance);
    fusion_->add(accel);
  }

  void onMag(const sensor_msgs::msg::MagneticField & msg)
  {
    const ImuRotator * rot = inputs_[kIcm].rotator.get();
    if (rot == nullptr || !rot->hasExtrinsic()) {
      return;  // mount TF not known yet
    }
    const Eigen::Vector3d raw(
      msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);
    const Eigen::Vector3d calibrated = mag_soft_iron_ * (raw - mag_hard_iron_);
    Measurement m;
    m.kind = Measurement::Kind::kMag;
    m.stamp = stampOrNow(msg.header.stamp);
    m.z = rot->rotation() * (mag_to_imu_ * calibrated);
    fusion_->add(m);
  }

  void onPublishTimer()
  {
    const double now_s = now().seconds();
    fusion_->process(now_s - reorder_delay_);

    const auto out = fusion_->output();
    if (!out) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for a stationary start to initialize (keep the rover still)");
      return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = toStamp(out->stamp);
    msg.header.frame_id = base_frame_;
    msg.orientation.w = out->q.w();
    msg.orientation.x = out->q.x();
    msg.orientation.y = out->q.y();
    msg.orientation.z = out->q.z();
    msg.orientation_covariance = covarianceToArray(out->q_cov);
    msg.angular_velocity.x = out->w.x();
    msg.angular_velocity.y = out->w.y();
    msg.angular_velocity.z = out->w.z();
    msg.angular_velocity_covariance = covarianceToArray(out->w_cov);
    // REP-145: acceleration includes gravity.
    msg.linear_acceleration.x = out->a.x();
    msg.linear_acceleration.y = out->a.y();
    msg.linear_acceleration.z = out->a.z();
    msg.linear_acceleration_covariance = covarianceToArray(out->a_cov);
    publisher_->publish(msg);
  }

  static diagnostic_msgs::msg::KeyValue kv(const std::string & k, const std::string & v)
  {
    diagnostic_msgs::msg::KeyValue out;
    out.key = k;
    out.value = v;
    return out;
  }

  static std::string num(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
  }

  void onDiagnosticsTimer()
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const double now_s = now().seconds();

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    int live = 0;
    int enabled = 0;
    for (int i = 0; i < 2; ++i) {
      const Input & in = inputs_[static_cast<std::size_t>(i)];
      if (!in.enabled) {
        continue;
      }
      ++enabled;
      const bool timed_out = fusion_->timedOut(i, now_s);
      live += timed_out ? 0 : 1;

      DiagnosticStatus st;
      st.name = "rover_imu_fusion/" + in.name;
      st.hardware_id = in.topic;
      st.level = timed_out ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
      st.message = timed_out ? "timed out" : "live";
      const double last = fusion_->lastStamp(i);
      st.values.push_back(kv("topic", in.topic));
      st.values.push_back(kv("age_s", last < 0.0 ? "never" : num(now_s - last)));
      if (const ImuEkf * f = fusion_->filter()) {
        const ImuCounters & c = f->counters(i);
        st.values.push_back(kv("applied", std::to_string(c.applied)));
        st.values.push_back(kv("rejected_total", std::to_string(c.rejected())));
        st.values.push_back(kv("rejected_stale", std::to_string(c.stale)));
        st.values.push_back(kv("rejected_nis", std::to_string(c.rejected_nis)));
        st.values.push_back(kv("rejected_saturated", std::to_string(c.rejected_saturated)));
        st.values.push_back(kv("rejected_stuck", std::to_string(c.rejected_stuck)));
        st.values.push_back(kv("forced_accepts", std::to_string(c.forced)));
        const auto k = static_cast<std::size_t>(i);
        const char axis[3] = {'x', 'y', 'z'};
        for (int a = 0; a < 3; ++a) {
          st.values.push_back(kv(std::string("gyro_bias_") + axis[a], num(f->state().bg[k](a))));
        }
        for (int a = 0; a < 3; ++a) {
          st.values.push_back(kv(std::string("accel_bias_") + axis[a], num(f->state().ba[k](a))));
        }
      }
      array.status.push_back(st);
    }

    DiagnosticStatus fs;
    fs.name = "rover_imu_fusion/filter";
    fs.hardware_id = "imu_fusion";
    if (!fusion_->initialized()) {
      fs.level = DiagnosticStatus::WARN;
      fs.message = "waiting for stationary initialization";
      fs.values.push_back(kv("init_restarts", std::to_string(fusion_->rejectedBeforeInit())));
    } else if (enabled > 0 && live == 0) {
      fs.level = DiagnosticStatus::ERROR;
      fs.message = "no live IMU";
    } else {
      fs.level = DiagnosticStatus::OK;
      fs.message = live < enabled ? "degraded: an IMU is timed out" : "ok";
      if (live < enabled) {
        fs.level = DiagnosticStatus::WARN;
      }
    }
    if (const auto out = fusion_->output()) {
      fs.values.push_back(kv("yaw_sigma_deg", num(std::sqrt(out->q_cov(2, 2)) * 180.0 / M_PI)));
      fs.values.push_back(
        kv(
          "roll_pitch_sigma_deg", num(
            std::sqrt(0.5 * (out->q_cov(0, 0) + out->q_cov(1, 1))) * 180.0 / M_PI)));
    }
    if (const ImuEkf * f = fusion_->filter()) {
      fs.values.push_back(kv("gravity_applied", std::to_string(f->gravityApplied())));
      fs.values.push_back(kv("gravity_skipped", std::to_string(f->gravitySkipped())));
    }
    fs.values.push_back(kv("buffer_dropped", std::to_string(fusion_->droppedFromBuffer())));
    array.status.push_back(fs);

    diag_publisher_->publish(array);
  }
};

}  // namespace rover_imu_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_imu_fusion::ImuFusionNode>());
  rclcpp::shutdown();
  return 0;
}
