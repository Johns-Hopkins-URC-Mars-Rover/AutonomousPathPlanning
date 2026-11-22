// ROS-free motion benchmark of the production ImuFusion class, not a second EKF.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include "rover_imu_fusion/imu_fusion.hpp"
using namespace rover_imu_fusion;

static void require(bool okay, const char *message) {
  if (!okay) throw std::runtime_error(message);
}
int main(int argc, char **argv) {
  try {
    std::map<std::string, double> values;
    for (int i = 1; i < argc; ++i) {
      const std::string arg(argv[i]); const auto equals = arg.find('=');
      if (equals == std::string::npos) throw std::invalid_argument("expected key=value");
      values[arg.substr(0, equals)] = std::stod(arg.substr(equals+1));
    }
    auto v = [&](const std::string &key) { return values.at(key); };
    FusionConfig cfg;
    auto &e = cfg.ekf;
    e.rate_walk=v("filter.rate_walk"); e.accel_walk=v("filter.accel_walk");
    e.init_attitude_sigma=v("filter.init_attitude_sigma");
    e.init_rate_sigma=v("filter.init_rate_sigma"); e.init_accel_sigma=v("filter.init_accel_sigma");
    e.gravity=v("filter.gravity"); e.gravity_sigma=v("filter.gravity_sigma");
    e.gravity_adapt=v("filter.gravity_adapt"); e.gravity_skip=v("filter.gravity_skip");
    e.nis_gate_3dof=v("filter.nis_gate_3dof"); e.nis_gate_1dof=v("filter.nis_gate_1dof");
    e.nis_max_consecutive=static_cast<int>(v("filter.nis_max_consecutive"));
    e.stuck_count=static_cast<int>(v("filter.stuck_count"));
    cfg.reorder_delay=v("reorder_delay"); cfg.imu_timeout=v("imu_timeout");
    cfg.gravity_update_period=v("gravity_update_period");
    cfg.init.duration=v("init.duration"); cfg.init.min_samples=static_cast<std::size_t>(v("init.min_samples"));
    cfg.init.gyro_std_max=v("init.gyro_std_max"); cfg.init.accel_std_max=v("init.accel_std_max");
    cfg.init.gyro_mean_max=v("init.gyro_mean_max"); cfg.init.accel_norm_tol=v("init.accel_norm_tol");
    cfg.init.gravity=e.gravity;
    for (int i=0; i<2; ++i) {
      const std::string prefix=i==0 ? "icm." : "zed.";
      e.gyro_bias_walk[i]=v(prefix+"gyro_bias_walk"); e.accel_bias_walk[i]=v(prefix+"accel_bias_walk");
      e.init_gyro_bias_sigma[i]=v(prefix+"init_gyro_bias_sigma");
      e.init_accel_bias_sigma[i]=v(prefix+"init_accel_bias_sigma");
      e.gyro_saturation[i]=v(prefix+"gyro_saturation"); e.accel_saturation[i]=v(prefix+"accel_saturation");
    }
    const bool zed_only=values.count("fixture.zed_only") && v("fixture.zed_only")!=0.;
    ImuFusion fusion(cfg);
    std::mt19937 rng(197);
    std::normal_distribution<double> noise(0.,1.);
    const double dt=.01;
    Eigen::Quaterniond truth=Eigen::Quaterniond::Identity();
    Eigen::Quaterniond initial_truth=truth;
    bool have_reference=false;
    Eigen::Quaterniond initial_estimate=truth;
    double sum_angle2=0., max_angle=0., sum_rate2=0.;
    std::size_t samples=0, dropout_samples=0;
    double initialization=-1.;
    for (int tick=0; tick<=1800; ++tick) {
      const double t=tick*dt;
      Eigen::Vector3d w=Eigen::Vector3d::Zero();
      if (t>=2. && t<6.) w.z()=std::acos(-1.)/8.;  // 90-degree yaw over 4 s
      else if(t>=6. && t<8.) w.x()=.15;
      else if(t>=8. && t<10.) w.y()=-.10;
      else if(t>=12.) w.z()=.10;
      // Truth at sample t advances by the preceding interval's rate.
      if (tick>0) {
        const double previous=(tick-1)*dt;
        Eigen::Vector3d previous_w=Eigen::Vector3d::Zero();
        if(previous>=2. && previous<6.) previous_w.z()=std::acos(-1.)/8.;
        else if(previous>=6. && previous<8.) previous_w.x()=.15;
        else if(previous>=8. && previous<10.) previous_w.y()=-.10;
        else if(previous>=12.) previous_w.z()=.10;
        truth=(truth*expQuat(previous_w*dt)).normalized();
      }
      if(t<16.) {
        for(int imu=0; imu<2; ++imu) {
          if(imu==0 && (zed_only || tick%2 || t>=12.)) continue; // 50 Hz ICM, drops out at 12 s
          const double gs=imu==0 ? .01 : .003;
          const double as=imu==0 ? .08 : .02;
          const Eigen::Vector3d bias=imu==0 ? Eigen::Vector3d(.02,-.01,.015) : Eigen::Vector3d::Zero();
          auto random=[&]() { return Eigen::Vector3d(noise(rng),noise(rng),noise(rng)); };
          Measurement gyro; gyro.imu=imu; gyro.stamp=1.+t;
          gyro.kind=Measurement::Kind::kGyro; gyro.z=w+bias+gs*random();
          gyro.R=gs*gs*Eigen::Matrix3d::Identity(); fusion.add(gyro);
          Measurement accel=gyro; accel.kind=Measurement::Kind::kAccel;
          accel.z=truth.conjugate()*Eigen::Vector3d(0.,0.,e.gravity)+as*random();
          accel.R=v(imu==0 ? "icm.accel_inflation" : "zed.accel_inflation")*as*as*Eigen::Matrix3d::Identity();
          fusion.add(accel);
        }
      }
      fusion.process(1.+t-cfg.reorder_delay);
      const auto out=fusion.output();
      if(!out) continue;
      if(initialization<0.) initialization=t;
      if(t<2.) { initial_truth=truth; initial_estimate=out->q; have_reference=true; }
      if(t>=2. && t<16.) {
        require(have_reference,"stationary initialization must finish before motion starts");
        // Compare relative orientation; account for the configured output lag.
        const Eigen::Quaterniond expected=(initial_truth.conjugate()*truth*expQuat(-w*cfg.reorder_delay)).normalized();
        const Eigen::Quaterniond estimated=(initial_estimate.conjugate()*out->q).normalized();
        const double error=logQuat(expected.conjugate()*estimated).norm();
        sum_angle2+=error*error; max_angle=std::max(max_angle,error);
        sum_rate2+=(out->w-w).squaredNorm(); ++samples;
        if(t>=12.) ++dropout_samples;
      }
      require(t<=16.+cfg.imu_timeout+.02,"complete IMU loss must stop output after timeout");
    }
    require(initialization>=0. && initialization<2.,"stationary initialization failed");
    require(samples>1000 && dropout_samples>300,"insufficient moving/dropout output");
    const double degrees=180./std::acos(-1.);
    const double angle_rmse=std::sqrt(sum_angle2/samples)*degrees;
    const double angle_max=max_angle*degrees;
    const double rate_rmse=std::sqrt(sum_rate2/samples);
    std::cout << "{\"initialization_s\":" << initialization << ",\"orientation_rmse_deg\":"
      << angle_rmse << ",\"orientation_max_deg\":" << angle_max
      << ",\"angular_rate_rmse_rad_s\":" << rate_rmse
      << ",\"moving_samples\":" << samples << ",\"single_imu_samples\":" << dropout_samples << "}\n";
    require(angle_rmse<3.,"synthetic orientation RMSE exceeds 3 degrees");
    require(angle_max<5.,"synthetic peak orientation error exceeds 5 degrees");
    require(rate_rmse<.05,"synthetic rate RMSE exceeds 0.05 rad/s");
  } catch(const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << '\n'; return 1;
  }
}
