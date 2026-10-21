#!/usr/bin/env python3
"""Check a stationary imu_compare.py CSV against what the IMU fusion claims.

Prints, per source (icm, zed, fused): the mean and standard deviation of
accel and gyro against the covariance the source publishes, the gravity norm,
and the fused yaw drift rate. Optionally saves plots.

Usage:
    python3 scripts/analyze_stationary.py results/imu_compare.csv
    python3 scripts/analyze_stationary.py run1.csv --skip 5 --plot results/stationary.png
"""

import argparse
import csv
import math

import numpy as np

SOURCES = ('icm', 'zed', 'fused')
AXES = ('x', 'y', 'z')


def load(path):
    data = {s: [] for s in SOURCES}
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['source'] in data:
                data[row['source']].append(row)
    out = {}
    for name, rows in data.items():
        if not rows:
            continue
        cols = {}
        for key in rows[0]:
            if key in ('source', 'frame_id'):
                continue
            cols[key] = np.array([float(r[key]) for r in rows])
        out[name] = cols
    return out


def stats(cols, skip):
    t = cols['stamp']
    keep = t >= t[0] + skip
    return {k: v[keep] for k, v in cols.items()}


def report(name, c):
    t = c['stamp']
    if len(t) < 2:
        print(f'\n=== {name}: fewer than 2 samples after --skip; record longer or lower --skip ===')
        return
    rate = (len(t) - 1) / (t[-1] - t[0])
    print(f'\n=== {name}: {len(t)} samples, {t[-1] - t[0]:.1f} s, {rate:.1f} Hz ===')
    norm = np.sqrt(c['ax'] ** 2 + c['ay'] ** 2 + c['az'] ** 2)
    print(f'|a| mean {norm.mean():.4f} std {norm.std():.4f} m/s^2 (gravity 9.8067)')
    print(f'{"":8}{"mean":>12}{"std":>12}{"var":>12}{"published cov":>16}{"var/cov":>10}')
    for kind, label, unit in (('a', 'accel', 'm/s^2'), ('g', 'gyro', 'rad/s')):
        cov_key = 'acc_cov' if kind == 'a' else 'gyro_cov'
        for ax in AXES:
            x = c[kind + ax]
            var = x.var(ddof=1)
            cov = c[f'{cov_key}_{ax}{ax}'].mean()
            ratio = var / cov if cov > 0 else float('nan')
            print(f'{label + "." + ax:8}{x.mean():12.5f}{x.std(ddof=1):12.5f}'
                  f'{var:12.3e}{cov:16.3e}{ratio:10.2f}   {unit}')


def yaw_drift(c):
    t = c['stamp'] - c['stamp'][0]
    yaw = np.unwrap(np.radians(c['yaw_deg']))
    slope = np.polyfit(t, np.degrees(yaw), 1)[0]
    print(f'\nFused yaw: start {np.degrees(yaw[0]):.2f} deg, end {np.degrees(yaw[-1]):.2f} deg, '
          f'drift {slope * 60:.3f} deg/min')
    print(f'Fused roll {c["roll_deg"].mean():.2f} +- {c["roll_deg"].std():.3f} deg, '
          f'pitch {c["pitch_deg"].mean():.2f} +- {c["pitch_deg"].std():.3f} deg')
    sig = np.degrees(np.sqrt(c['ori_cov_zz']))
    print(f'Fused yaw sigma: start {sig[0]:.2f} deg, end {sig[-1]:.2f} deg')


def plot(data, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    t0 = min(c['stamp'][0] for c in data.values())
    for name, c in data.items():
        t = c['stamp'] - t0
        axes[0].plot(t, c['gz'], label=name, lw=0.6)
        axes[1].plot(t, np.sqrt(c['ax'] ** 2 + c['ay'] ** 2 + c['az'] ** 2), label=name, lw=0.6)
    axes[0].set_ylabel('gyro z [rad/s]')
    axes[1].set_ylabel('|accel| [m/s^2]')
    if 'fused' in data:
        f = data['fused']
        axes[2].plot(f['stamp'] - t0, f['yaw_deg'], label='yaw')
        axes[2].plot(f['stamp'] - t0, f['roll_deg'], label='roll')
        axes[2].plot(f['stamp'] - t0, f['pitch_deg'], label='pitch')
    axes[2].set_ylabel('fused angle [deg]')
    axes[2].set_xlabel('time [s]')
    for a in axes:
        a.legend(loc='upper right')
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f'\nSaved plot to {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('csv')
    parser.add_argument('--skip', type=float, default=5.0,
                        help='seconds to drop at the start (filter init) [s]')
    parser.add_argument('--plot', help='save plots to this png')
    args = parser.parse_args()

    data = load(args.csv)
    missing = [s for s in SOURCES if s not in data]
    if missing:
        print(f'Warning: no rows for {", ".join(missing)}')
    data = {n: stats(c, args.skip) for n, c in data.items()}
    data = {n: c for n, c in data.items() if len(c['stamp']) >= 2}
    for name in SOURCES:
        if name in data:
            report(name, data[name])
    if 'fused' in data:
        yaw_drift(data['fused'])
    if args.plot:
        plot(data, args.plot)


if __name__ == '__main__':
    main()
