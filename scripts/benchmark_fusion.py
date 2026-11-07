#!/usr/bin/env python3
"""ROS-free noisy IMU motion benchmark.

Runs the real production ImuFusion C++ core using configuration values read from
imu_fusion.yaml. All motion/noise/covariance inputs are SYNTHETIC. This does not
execute the camera+IMU robot_localization EKF or reproduce its original cause.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

import check_localization

ROOT = Path(__file__).resolve().parents[1]


def config_values(path):
    code = 'require "json"; p=YAML.safe_load(File.read(ARGV[0])); puts JSON.generate(p["imu_fusion"]["ros__parameters"])'
    result = subprocess.run(['ruby', '-ryaml', '-e', code, str(path)],
                            capture_output=True, text=True, check=True)
    def flatten(value, prefix=''):
        output = {}
        for key, item in value.items():
            name = prefix+key
            if isinstance(item, dict):
                output.update(flatten(item, name+'.'))
            elif isinstance(item, (float, int)) and not isinstance(item, bool):
                output[name] = item
        return output
    return flatten(json.loads(result.stdout))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--eigen-include', type=Path)
    args = parser.parse_args()
    compiler = shutil.which('clang++') or shutil.which('c++')
    if not compiler:
        parser.error('C++17 compiler required')
    eigen = check_localization.find_eigen(args.eigen_include)
    config = ROOT/'src/rover_imu_fusion/config/imu_fusion.yaml'
    values = config_values(config)
    include = ROOT/'src/rover_imu_fusion/include'
    sources = [ROOT/'src/rover_imu_fusion/test/motion_benchmark.cpp']
    sources += [ROOT/f'src/rover_imu_fusion/src/{n}.cpp'
                for n in ('imu_ekf', 'imu_fusion', 'imu_initializer', 'imu_transform')]
    report = dict(input_kind='synthetic; not a hardware recording',
                  config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
                  scope='IMU core orientation/rate only; no position EKF',
                  gyro_noise_std_rad_s=dict(zed=.003, icm=.01),
                  accel_noise_std_m_s2=dict(zed=.02, icm=.08),
                  assumptions='Both sensors aligned in base_link; gyro biases ZED=0, ICM=[0.02,-0.01,0.015] rad/s; no translational acceleration.',
                  thresholds=dict(orientation_rmse_deg=3., orientation_peak_deg=5., angular_rate_rmse_rad_s=.05),
                  motion={})
    with tempfile.TemporaryDirectory(prefix='rover-motion-') as directory:
        binary = Path(directory)/'motion_benchmark'
        print('Building the real IMU fusion core...', flush=True)
        subprocess.run([compiler, '-O1', '-DNDEBUG', '-std=c++17', f'-I{eigen}', f'-I{include}',
                        *map(str, sources), '-o', str(binary)], check=True)
        for mode, zed_only in [('zed_only', 1), ('two_imus_then_icm_dropout', 0)]:
            command = [str(binary), *(f'{key}={value}' for key, value in values.items()),
                       f'fixture.zed_only={zed_only}']
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            if result.returncode:
                print(result.stdout, end='')
                raise RuntimeError(f'{mode}: {result.stderr.strip()}')
            metrics = json.loads(result.stdout)
            report['motion'][mode] = metrics
            print(f'PASS {mode}: orientation RMSE {metrics["orientation_rmse_deg"]:.3f} deg; '
                  f'peak {metrics["orientation_max_deg"]:.3f} deg; '
                  f'rate RMSE {metrics["angular_rate_rmse_rad_s"]:.5f} rad/s')
    print('LIMIT: IMU orientation/rate only. Run scripts/test_localization_container.sh for the camera+gyro position EKF.')
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2)+'\n')
        print(f'Report: {args.report}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
