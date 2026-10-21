// Replays an imu_compare.py CSV through the ROS-free ImuFusion pipeline so the
// filter can be tuned offline.
//
// Only the "icm" and "zed" rows are used; "fused" rows from the recording are
// ignored. Both IMUs are assumed to already be in base_link axes (no mount
// rotation), and only the diagonal of each covariance is in the CSV.
//
// Build (from this directory):
//   g++ -O2 -std=c++17 -I/usr/include/eigen3 -I../include replay_csv.cpp \
//       ../src/imu_ekf.cpp ../src/imu_fusion.cpp ../src/imu_initializer.cpp \
//       ../src/imu_transform.cpp -o /tmp/replay_csv
//
// Usage:
//   replay_csv run.csv [--skip 5] [--rate-walk 1.0] [--accel-walk 5.0]
//                      [--gravity-sigma 0.5] [--no-icm] [--no-zed]
//                      [--accel-inflation 1.5] [--out fused.csv]

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "rover_imu_fusion/imu_fusion.hpp"

using namespace rover_imu_fusion;

namespace
{
struct Sample
{
  int imu;
  double stamp;
  Eigen::Vector3d a, g, a_var, g_var;
};

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    out.push_back(cell);
  }
  return out;
}

std::vector<Sample> load(const std::string & path, bool use_icm, bool use_zed)
{
  std::ifstream f(path);
  if (!f) {
    std::cerr << "cannot open " << path << "\n";
    std::exit(1);
  }
  std::string line;
  std::getline(f, line);
  std::map<std::string, size_t> col;
  const auto header = split(line);
  for (size_t i = 0; i < header.size(); ++i) {
    col[header[i]] = i;
  }
  std::vector<Sample> out;
  while (std::getline(f, line)) {
    const auto c = split(line);
    const std::string & src = c[col["source"]];
    int imu;
    if (src == "icm" && use_icm) {
      imu = 0;
    } else if (src == "zed" && use_zed) {
      imu = 1;
    } else {
      continue;
    }
    auto v = [&](const char * k) {return std::stod(c[col[k]]);};
    Sample s;
    s.imu = imu;
    s.stamp = v("stamp");
    s.a = {v("ax"), v("ay"), v("az")};
    s.g = {v("gx"), v("gy"), v("gz")};
    s.a_var = {v("acc_cov_xx"), v("acc_cov_yy"), v("acc_cov_zz")};
    s.g_var = {v("gyro_cov_xx"), v("gyro_cov_yy"), v("gyro_cov_zz")};
    out.push_back(s);
  }
  std::stable_sort(out.begin(), out.end(), [](const Sample & x, const Sample & y) {
      return x.stamp < y.stamp;
    });
  return out;
}

struct Acc
{
  std::vector<double> v;
  void add(double x) {v.push_back(x);}
  double mean() const
  {
    double s = 0;
    for (double x : v) {s += x;}
    return v.empty() ? 0 : s / static_cast<double>(v.size());
  }
  double var() const
  {
    const double m = mean();
    double s = 0;
    for (double x : v) {s += (x - m) * (x - m);}
    return v.size() < 2 ? 0 : s / static_cast<double>(v.size() - 1);
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: replay_csv run.csv [options]\n";
    return 1;
  }
  const std::string path = argv[1];
  double skip = 5.0, inflation = 1.5;
  bool use_icm = true, use_zed = true;
  std::string out_path;

  FusionConfig cfg;
  auto & e = cfg.ekf;
  // Same values as config/imu_fusion.yaml.
  e.rate_walk = 1.0;
  e.accel_walk = 5.0;
  e.init_attitude_sigma = 0.05;
  e.init_rate_sigma = 0.05;
  e.init_accel_sigma = 0.5;
  e.gravity_sigma = 0.5;
  e.stuck_count = 20;
  e.gyro_bias_walk = {1e-4, 1e-5};
  e.accel_bias_walk = {1e-3, 1e-4};
  e.init_gyro_bias_sigma = {0.01, 0.002};
  e.init_accel_bias_sigma = {0.1, 0.05};
  e.gyro_saturation = {33.0, 0.0};
  e.accel_saturation = {149.0, 0.0};

  for (int i = 2; i < argc; ++i) {
    const std::string k = argv[i];
    auto next = [&]() {return std::stod(argv[++i]);};
    if (k == "--skip") {skip = next();}
    else if (k == "--rate-walk") {e.rate_walk = next();}
    else if (k == "--accel-walk") {e.accel_walk = next();}
    else if (k == "--gravity-sigma") {e.gravity_sigma = next();}
    else if (k == "--icm-gyro-bias-walk") {e.gyro_bias_walk[0] = next();}
    else if (k == "--icm-accel-bias-walk") {e.accel_bias_walk[0] = next();}
    else if (k == "--accel-inflation") {inflation = next();}
    else if (k == "--no-icm") {use_icm = false;}
    else if (k == "--no-zed") {use_zed = false;}
    else if (k == "--out") {out_path = argv[++i];}
    else {std::cerr << "unknown option " << k << "\n"; return 1;}
  }

  const auto samples = load(path, use_icm, use_zed);
  if (samples.empty()) {
    std::cerr << "no icm/zed rows\n";
    return 1;
  }

  ImuFusion fusion(cfg);
  std::ofstream out;
  if (!out_path.empty()) {
    out.open(out_path);
    out << "source,stamp,frame_id,ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,qx,qy,qz,qw,"
      "acc_cov_xx,acc_cov_yy,acc_cov_zz,gyro_cov_xx,gyro_cov_yy,gyro_cov_zz,"
      "ori_cov_xx,ori_cov_yy,ori_cov_zz\n";
    out.precision(12);
  }

  Acc ax[3], gx[3], acov[3], gcov[3], anorm, roll, pitch, yaw, yawsig, t;
  const double t0 = samples.front().stamp, t_end = samples.back().stamp;
  const double dt = 0.01;  // 100 Hz publish timer
  size_t next = 0;
  double first_out = -1.0;
  for (double now = t0; now <= t_end; now += dt) {
    while (next < samples.size() && samples[next].stamp <= now) {
      const Sample & s = samples[next++];
      Measurement g;
      g.kind = Measurement::Kind::kGyro;
      g.imu = s.imu;
      g.stamp = s.stamp;
      g.z = s.g;
      g.R = Eigen::Matrix3d(s.g_var.asDiagonal());
      fusion.add(g);
      Measurement a;
      a.kind = Measurement::Kind::kAccel;
      a.imu = s.imu;
      a.stamp = s.stamp;
      a.z = s.a;
      a.R = Eigen::Matrix3d(inflation * s.a_var.asDiagonal());
      fusion.add(a);
    }
    fusion.process(now - cfg.reorder_delay);
    const auto o = fusion.output();
    if (!o) {
      continue;
    }
    if (first_out < 0) {
      first_out = now;
    }
    const auto & q = o->q;
    const double r = std::atan2(
      2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
    const double p = std::asin(std::clamp(2 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0));
    const double y = std::atan2(
      2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    const double rd = 180.0 / M_PI;
    if (out.is_open()) {
      out << "fused," << o->stamp << ",base_link," << o->a.x() << ',' << o->a.y() << ','
          << o->a.z() << ',' << o->w.x() << ',' << o->w.y() << ',' << o->w.z() << ','
          << r * rd << ',' << p * rd << ',' << y * rd << ',' << q.x() << ',' << q.y() << ','
          << q.z() << ',' << q.w() << ',' << o->a_cov(0, 0) << ',' << o->a_cov(1, 1) << ','
          << o->a_cov(2, 2) << ',' << o->w_cov(0, 0) << ',' << o->w_cov(1, 1) << ','
          << o->w_cov(2, 2) << ',' << o->q_cov(0, 0) << ',' << o->q_cov(1, 1) << ','
          << o->q_cov(2, 2) << '\n';
    }
    if (now - first_out < skip) {
      continue;
    }
    for (int k = 0; k < 3; ++k) {
      ax[k].add(o->a[k]);
      gx[k].add(o->w[k]);
      acov[k].add(o->a_cov(k, k));
      gcov[k].add(o->w_cov(k, k));
    }
    anorm.add(o->a.norm());
    roll.add(r * rd);
    pitch.add(p * rd);
    yaw.add(y * rd);
    yawsig.add(std::sqrt(o->q_cov(2, 2)) * rd);
    t.add(now - t0);
  }

  if (t.v.empty()) {
    std::cerr << "filter never initialized (needs a stationary start)\n";
    return 1;
  }
  // Least-squares yaw slope.
  const double tm = t.mean(), ym = yaw.mean();
  double num = 0, den = 0;
  for (size_t i = 0; i < t.v.size(); ++i) {
    num += (t.v[i] - tm) * (yaw.v[i] - ym);
    den += (t.v[i] - tm) * (t.v[i] - tm);
  }

  std::printf("rate_walk %.4g accel_walk %.4g gravity_sigma %.3g\n",
    e.rate_walk, e.accel_walk, e.gravity_sigma);
  std::printf("%-8s%12s%12s%14s%10s\n", "", "mean", "std", "published", "var/cov");
  const char * names = "xyz";
  for (int k = 0; k < 3; ++k) {
    std::printf("accel.%c %12.5f%12.5f%14.3e%10.2f\n", names[k], ax[k].mean(),
      std::sqrt(ax[k].var()), acov[k].mean(), ax[k].var() / acov[k].mean());
  }
  for (int k = 0; k < 3; ++k) {
    std::printf("gyro.%c  %12.5f%12.5f%14.3e%10.2f\n", names[k], gx[k].mean(),
      std::sqrt(gx[k].var()), gcov[k].mean(), gx[k].var() / gcov[k].mean());
  }
  std::printf("|a| mean %.4f std %.4f; roll %.2f+-%.3f pitch %.2f+-%.3f deg\n",
    anorm.mean(), std::sqrt(anorm.var()), roll.mean(), std::sqrt(roll.var()),
    pitch.mean(), std::sqrt(pitch.var()));
  std::printf("yaw drift %.3f deg/min, yaw sigma %.2f -> %.2f deg\n", num / den * 60.0,
    yawsig.v.front(), yawsig.v.back());
  return 0;
}
