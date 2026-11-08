#!/usr/bin/env python3
"""Rebuild and check localization without ROS; add --ros for sourced Ubuntu checks.

Uses existing dependencies only. Builds native tests in a temporary directory.
No hardware drivers, Git changes, network installers or live motion are started.
"""
import argparse
import ast
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import trace
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ('rover_localization', 'rover_imu_fusion', 'rover_description',
            'rover_bringup', 'l76x_gps_interface')


def execute(label, command, results, timeout=180):
    print(f'CHECK: {label}', flush=True)
    started = time.monotonic()
    try:
        code = subprocess.run([str(v) for v in command], cwd=ROOT,
                              env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'),
                              timeout=timeout, check=False).returncode
    except subprocess.TimeoutExpired:
        code = 124
        print(f'FAIL: {label} timed out after {timeout} seconds', file=sys.stderr)
    except OSError as error:
        code = 127
        print(f'FAIL: {label}: {error}', file=sys.stderr)
    results.append(dict(check=label, exit_code=code,
                        seconds=round(time.monotonic()-started, 3)))
    return code == 0


def find_eigen(explicit=None):
    candidates = [Path(explicit)] if explicit else [
        Path('/usr/include/eigen3'), Path('/usr/local/include/eigen3'),
        Path('/opt/homebrew/include/eigen3'), Path('/opt/homebrew/opt/eigen/include/eigen3'),
        *sorted(Path('/opt/homebrew/Cellar/eigen').glob('*/include/eigen3')),
    ]
    for path in candidates:
        if (path / 'Eigen/Core').is_file():
            return path
    raise RuntimeError('Eigen headers not found. Supply --eigen-include /path/to/include/eigen3; no dependencies were installed.')


def source_syntax():
    for folder in [ROOT/'scripts', *(ROOT/'src'/p for p in PACKAGES)]:
        for path in folder.rglob('*.py'):
            ast.parse(path.read_text(), filename=str(path))
        for path in folder.rglob('package.xml'):
            ET.parse(path)
    for folder, pattern in [('urdf', '*.urdf'), ('launch', '*.launch')]:
        for path in (ROOT/'src/rover_description'/folder).glob(pattern):
            ET.parse(path)
    json.loads((ROOT/'src/rover_description/config/sensor_mounts.example.json').read_text())


def python_tests(coverage=False):
    suites = [unittest.TestLoader().discover(
        str(ROOT/'src/rover_localization/test'), pattern='test_*.py'),
        unittest.TestLoader().discover(
        str(ROOT/'src/l76x_gps_interface/test'), pattern='test_gps_contract.py')]
    suite = unittest.TestSuite(suites)
    test_runner = unittest.TextTestRunner(verbosity=1)
    tracer = trace.Trace(count=True, trace=False) if coverage else None
    result = tracer.runfunc(test_runner.run, suite) if tracer else test_runner.run(suite)
    measurements = {}
    if tracer:
        counts = tracer.results().counts
        for relative in ('src/rover_localization/scripts/sensor_contracts.py',
                         'src/rover_localization/scripts/sensor_inputs.py',
                         'src/rover_localization/scripts/zed_imu_tf.py',
                         'src/l76x_gps_interface/l76x_gps_interface/nmea.py'):
            path = str(ROOT/relative)
            lines = {line for line in trace._find_executable_linenos(path)
                     if isinstance(line, int) and line > 0}
            covered = sum((path, line) in counts for line in lines)
            percent = 100.*covered/len(lines)
            measurements[relative] = dict(covered=covered, statements=len(lines), percent=round(percent, 1))
            print(f'COVERAGE: {relative}: {covered}/{len(lines)} ({percent:.1f}%)')
    return (result.wasSuccessful() and all(m['percent'] >= 80 for m in measurements.values()),
            result.testsRun, measurements)


def native_tests(compiler, eigen, results):
    with tempfile.TemporaryDirectory(prefix='rover-check-') as temp:
        build = Path(temp)
        include = ROOT/'src/rover_imu_fusion/include'
        flags = [compiler, '-O1', '-DNDEBUG', '-std=c++17', f'-I{eigen}', f'-I{include}']
        objects = []
        for name in ('imu_ekf', 'imu_fusion', 'imu_initializer', 'imu_transform'):
            source = ROOT/f'src/rover_imu_fusion/src/{name}.cpp'
            obj = build/(name+'.o')
            if not execute(f'compile {name}', [*flags, '-c', source, '-o', obj], results):
                return
            objects.append(obj)
        targets = [('imu_input_safety', ROOT/'src/rover_imu_fusion/test/test_input_safety.cpp'),
                   ('imu_csv_replay', ROOT/'src/rover_imu_fusion/tools/replay_csv.cpp')]
        for name, source in targets:
            binary = build/name
            if not execute(f'build {name}', [*flags, source, *objects, '-o', binary], results):
                continue
            command = [binary] if name == 'imu_input_safety' else [
                sys.executable, '-B', ROOT/'src/rover_imu_fusion/test/test_replay_csv.py', binary]
            execute(name, command, results)
        binary = build/'motion_model'
        if execute('build Release motion checks', [compiler, '-std=c++17', '-DNDEBUG',
                   f'-I{ROOT / "src/rover_localization/include"}',
                   ROOT/'src/rover_localization/test/test_simulation_model.cpp', '-o', binary], results):
            execute('Release motion model', [binary], results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--eigen-include', type=Path)
    parser.add_argument('--cxx', default=shutil.which('clang++') or shutil.which('c++'))
    parser.add_argument('--coverage', action='store_true', help='require 80%% trace line coverage for new sensor helpers')
    parser.add_argument('--ros', action='store_true', help='also build and test ROS packages in the sourced workspace')
    parser.add_argument('--report', type=Path, help='optional JSON result path')
    args = parser.parse_args()
    results = []
    try:
        eigen = find_eigen(args.eigen_include)
        if not args.cxx:
            raise RuntimeError('A C++17 compiler is required (--cxx).')
        if args.ros and not all(shutil.which(tool) for tool in ('ros2', 'colcon')):
            raise RuntimeError('--ros requires ROS 2/colcon: run on Ubuntu after sourcing ROS Humble and the workspace.')
        source_syntax()
        results.append(dict(check='Python/XML/JSON syntax', exit_code=0))
        okay, count, coverage = python_tests(args.coverage)
        results.append(dict(check='Python tests', tests=count, exit_code=0 if okay else 1, coverage=coverage))
        execute('Ruby configuration contracts', ['ruby', ROOT/'src/rover_localization/test/test_localization_contract.rb'], results)
        yaml_files = [str(path) for p in PACKAGES for path in (ROOT/'src'/p/'config').glob('*.yaml')]
        execute('YAML syntax', ['ruby', '-ryaml', '-e', 'ARGV.each { |p| YAML.safe_load(File.read(p)) }', *yaml_files], results)
        execute('smoke shell syntax', ['bash', '-n', ROOT/'src/rover_localization/scripts/smoke_test.sh'], results)
        native_tests(args.cxx, eigen, results)
        if args.ros:
            if execute('ROS package build', ['colcon', 'build', '--symlink-install', '--packages-select', *PACKAGES], results, timeout=1800):
                # A child shell sources the just-built overlay; no host environment is modified.
                execute('ROS package tests', ['bash', '-c',
                    'source install/setup.bash && colcon test --packages-select rover_localization rover_imu_fusion l76x_gps_interface --event-handlers console_direct+'], results, timeout=900)
                execute('ROS test results', ['colcon', 'test-result', '--verbose'], results)
        else:
            print('NOT RUN: ROS graph/build/GoogleTest/hardware checks (use --ros on sourced Ubuntu).')
    except (RuntimeError, SyntaxError, ValueError, ET.ParseError, OSError) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        results.append(dict(check='verification prerequisites/source', exit_code=1, error=str(error)))
    passed = bool(results) and all(item['exit_code'] == 0 for item in results)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(dict(passed=passed, ros_requested=args.ros, results=results), indent=2)+'\n')
    print('PASS: requested checks completed' if passed else 'FAIL: one or more requested checks failed')
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
