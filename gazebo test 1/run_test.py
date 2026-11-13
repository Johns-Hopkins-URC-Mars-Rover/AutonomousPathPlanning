#!/usr/bin/env python3
"""Run the existing headless test and retain its console output and measurements."""
import csv
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
LIMITS = {
    'gps_conversion_error_m': ('m', '< 2.0'),
    'truth_displacement_m': ('m', '0.10 < value < 1.0'),
    'forward_m': ('m', '-1.0 < value < -0.10'),
    'local_delta_error_m': ('m', '< 0.4'),
    'reverse_forward_axis_m': ('m', '0.10 < value < 1.0'),
    'turn_yaw_rad': ('rad', '-1.6 < value < -0.2'),
    'obstacle_baseline_cost': ('cost', '0 <= value < 90'),
    'obstacle_marked_cost': ('cost', '>= 90'),
}


def parse(text, exit_code):
    rows = []
    profile = None
    for line in text.splitlines():
        match = re.match(r'Summary: (\d+) tests, (\d+) errors, (\d+) failures, (\d+) skipped', line)
        if match:
            for name, value in zip(('tests', 'errors', 'failures', 'skipped'), match.groups()):
                rows.append(('package_tests', name, value, 'count', '', ''))
        try:
            data = json.loads(line)
        except (ValueError, TypeError):
            data = None
        if isinstance(data, dict):
            if 'streams' in data:
                profile = 'motion' if profile is None else 'navigation'
                for name, value in data['streams'].items():
                    rows.append((profile, 'stream_count_' + name, value, 'messages', '>= 3', ''))
            for name, value in data.items():
                if name in LIMITS:
                    unit, expected = LIMITS[name]
                    rows.append((profile or 'unknown', name, value, unit, expected, ''))
        if line.startswith('PASS simulation profile: '):
            name = line.split(': ', 1)[1]
            rows.append((name, 'profile', 'PASS', '', 'PASS', ''))
        if line.startswith('PASS Nav2 reachable goal,'):
            rows.append(('navigation', 'navigation_checks', 'PASS', '', 'PASS',
                         'Goal, cancellation, outside-map handling and obstacle marking; see log'))
    rows.append(('overall', 'process_exit_code', exit_code, '', '0', 'Nonzero means the run did not complete successfully'))
    return rows


def main():
    stamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
    output = HERE / 'runs' / stamp
    output.mkdir(parents=True)
    command = ['bash', str(ROOT / 'scripts' / 'test_simulation_container.sh'), *sys.argv[1:]]
    print(f'Output folder: {output}', flush=True)
    print('Command: ' + ' '.join(command), flush=True)
    lines = []
    with (output / 'console.log').open('w', encoding='utf-8') as log:
        try:
            process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, errors='replace', bufsize=1)
            for line in process.stdout:
                print(line, end='', flush=True)
                log.write(line)
                log.flush()
                lines.append(line)
            code = process.wait()
        except KeyboardInterrupt:
            process.terminate()
            code = process.wait()
            if code == 0:
                code = 130
        except OSError as error:
            message = f'Unable to start test: {error}\n'
            print(message, file=sys.stderr)
            log.write(message)
            code = 1
    with (output / 'results.csv').open('w', encoding='utf-8', newline='') as result:
        writer = csv.writer(result)
        writer.writerow(('profile', 'metric', 'observed_value', 'unit', 'expected', 'notes'))
        writer.writerows(parse(''.join(lines), code))
    print(f'CSV: {output / "results.csv"}\nLog: {output / "console.log"}', flush=True)
    return code if code >= 0 else 128 - code


if __name__ == '__main__':
    sys.exit(main())
