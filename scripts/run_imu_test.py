#!/usr/bin/env python3
"""Run a stationary or moving IMU test end to end.

Starts the CSV logger (imu_compare.py) and a bag recording, runs for a set time,
stops both with SIGINT so the bag closes cleanly, checks that the bag and CSV
are readable, and for a stationary run prints the analysis. Everything goes in
one folder: <out>/<mode>_<timestamp>/ (bag/, imu_compare.csv, run_info.txt,
and stationary.png for a stationary run).

The rover stack must already be running (ros2 launch rover_bringup rover.launch.py).
Source ROS and the workspace first.

Usage:
    python3 scripts/run_imu_test.py stationary
    python3 scripts/run_imu_test.py moving --duration 90 --notes "2 slow turns, then figure 8"
"""

import argparse
import csv
import datetime
import json
import os
import signal
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
BAG_TOPICS = ['/imu/data_raw', '/zed/zed_node/imu/data', '/imu/data', '/tf', '/tf_static']
DEFAULT_DURATION = {'stationary': 90.0, 'moving': 90.0}
STILL_START = 5.0   # s the rover must be still at the start of every run
STOP_TIMEOUT = 30.0  # s to wait for a process to exit after SIGINT


def start(cmd, log=None):
    # Own session so a terminal Ctrl-C does not hit the children directly; this
    # script forwards the signal and waits for them to exit.
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop(proc, name):
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=STOP_TIMEOUT)
    except subprocess.TimeoutExpired:
        print(f'{name} did not exit within {STOP_TIMEOUT:.0f} s; terminating')
        proc.terminate()
        proc.wait(timeout=10)


def countdown(seconds):
    for left in range(int(seconds), 0, -1):
        print(f'\rStarting in {left:2d} s (step away, keep the rover still)  ', end='', flush=True)
        time.sleep(1)
    print()


def wait_for_duration(duration, mode, procs):
    t0 = time.monotonic()
    announced_go = False
    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= duration:
            return True
        for name, p in procs.items():
            if p.poll() is not None:
                print(f'\n{name} exited early (code {p.returncode})')
                return False
        if mode == 'moving' and not announced_go and elapsed >= STILL_START:
            print('\a>>> GO: start the motion routine now <<<')
            announced_go = True
        left = duration - elapsed
        print(f'\r{elapsed:6.0f} s elapsed, {left:5.0f} s left '
              f'{"(still)" if elapsed < STILL_START else ""}   ', end='', flush=True)
        time.sleep(1)


def check_bag(bag_dir):
    info = subprocess.run(['ros2', 'bag', 'info', str(bag_dir)],
                          capture_output=True, text=True)
    ok = info.returncode == 0 and (bag_dir / 'metadata.yaml').exists()
    print(info.stdout if ok else f'BAG PROBLEM: {info.stderr.strip() or info.stdout.strip()}')
    return ok


def check_csv(path):
    counts = Counter()
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            counts[row['source']] += 1
    print('CSV rows: ' + ', '.join(f'{s}={counts.get(s, 0)}' for s in ('icm', 'zed', 'fused')))
    return all(counts.get(s, 0) > 0 for s in ('icm', 'zed', 'fused')), counts


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('mode', choices=('stationary', 'moving'))
    parser.add_argument('--duration', type=float, help='seconds to record '
                        f'(default {DEFAULT_DURATION})')
    parser.add_argument('--out', default=str(Path.home() / 'rover_data'),
                        help='folder that holds the run folders')
    parser.add_argument('--countdown', type=float, default=10.0,
                        help='seconds to wait before starting [s]')
    parser.add_argument('--notes', default='', help='what you plan to do (saved in run_info.txt)')
    parser.add_argument('--no-bag', action='store_true', help='CSV only, skip the bag')
    parser.add_argument('--skip', type=float, default=5.0, help='seconds dropped by the analysis')
    args = parser.parse_args()

    duration = args.duration or DEFAULT_DURATION[args.mode]
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = Path(args.out) / f'{args.mode}_{stamp}'
    run_dir.mkdir(parents=True)
    csv_path = run_dir / 'imu_compare.csv'
    bag_dir = run_dir / 'bag'
    print(f'Run folder: {run_dir}')

    countdown(args.countdown)
    print(f'Recording {args.mode} for {duration:.0f} s. Ctrl-C ends it early and still saves.')

    log = open(run_dir / 'imu_compare.log', 'w')
    procs = {'imu_compare': start(
        [sys.executable, str(SCRIPTS / 'imu_compare.py'), '--output', str(csv_path),
         '--period', '30'], log)}
    if not args.no_bag:
        procs['ros2 bag'] = start(
            ['ros2', 'bag', 'record', '-o', str(bag_dir), *BAG_TOPICS], log)

    started = datetime.datetime.now().isoformat(timespec='seconds')
    finished_normally = True
    try:
        finished_normally = wait_for_duration(duration, args.mode, procs)
    except KeyboardInterrupt:
        print('\nStopping early')
        finished_normally = False
    print('\nStopping recorders...')
    for name, p in procs.items():
        stop(p, name)
    log.close()

    (run_dir / 'run_info.txt').write_text(json.dumps({
        'mode': args.mode, 'started': started, 'planned_duration_s': duration,
        'ended_early': not finished_normally, 'notes': args.notes,
        'topics': BAG_TOPICS,
    }, indent=2) + '\n')

    print('\n--- Checks ---')
    ok = True
    if not args.no_bag:
        ok &= check_bag(bag_dir)
    csv_ok, counts = check_csv(csv_path)
    ok &= csv_ok
    if not csv_ok:
        missing = [s for s in ('icm', 'zed', 'fused') if not counts.get(s)]
        print(f'No data for: {", ".join(missing)}. Is the rover stack running?')

    if args.mode == 'stationary' and counts.get('fused'):
        print('\n--- Stationary analysis ---')
        subprocess.run([sys.executable, str(SCRIPTS / 'analyze_stationary.py'), str(csv_path),
                        '--skip', str(args.skip), '--plot', str(run_dir / 'stationary.png')])
    elif args.mode == 'moving':
        print('\nMoving run saved. Tune with tools/replay_csv (see TESTING.md); '
              'the stationary analysis does not apply.')

    print(f'\nSaved to {run_dir}')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
