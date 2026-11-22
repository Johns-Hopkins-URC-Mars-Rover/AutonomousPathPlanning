"""Native replay contract regression, including Python CSV writer's CRLF rows."""
import csv
from pathlib import Path
import subprocess
import sys
import tempfile


def run_cases(executable):
    with tempfile.TemporaryDirectory(prefix='rover_replay_') as directory:
        root = Path(directory)
        axes = ['xx', 'xy', 'xz', 'yx', 'yy', 'yz', 'zx', 'zy', 'zz']
        columns = ['source', 'stamp', 'frame_id', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
        columns += [kind + '_cov_' + axis for kind in ('acc', 'gyro') for axis in axes]
        for name, frame, legacy in [('valid', 'base_link', False),
                                     ('wrong_frame', 'zed_imu_link', False),
                                     ('legacy', 'base_link', True)]:
            cols = [c for c in columns if not legacy or '_cov_' not in c or c[-2:] in ('xx', 'yy', 'zz')]
            path = root / (name + '.csv')
            with path.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=cols)
                writer.writeheader()
                for i in range(300):
                    row = {c: 0. for c in cols}
                    row.update(source='zed', stamp=1.+i*.01, frame_id=frame, az=9.80665)
                    for c in cols:
                        if '_cov_' in c:
                            row[c] = .001 if c[-2] == c[-1] else 0.
                    writer.writerow(row)
            result = subprocess.run([executable, str(path), '--no-icm', '--skip', '0'],
                                    text=True, capture_output=True, timeout=30)
            if (result.returncode == 0) != (name == 'valid'):
                raise AssertionError(f'{name}: unexpected exit {result.returncode}: {result.stderr}')
            print(f'PASS: replay {name} contract')


if __name__ == '__main__':
    run_cases(sys.argv[1])
