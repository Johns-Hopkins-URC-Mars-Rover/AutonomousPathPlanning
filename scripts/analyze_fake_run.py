#!/usr/bin/env python3
"""EXPECTED vs ACTUAL for a fake-sensor run. Usage: analyze_fake_run.py RUN_DIR

Expected behaviour is computed from the path spec alone (pure geometry: the
rover starts at the origin facing east and follows the drive/turn segments).
Actual behaviour is what the local EKF (odom frame) and global EKF (map frame)
reported. Writes into RUN_DIR:
  summary.txt        checks with PASS/FAIL, per-segment expected vs actual, log warnings
  segments.csv       the per-segment table in machine-readable form
  path_compare.png   XY path, yaw over time and position error over time

Frames: the local EKF starts at the true start, so it is compared directly. The
global EKF's map origin is the FIRST noisy GPS fix, so it is compared with the
expected pose minus that fix's error (a constant offset no filter can know).
"""
import csv
import math
import os
import re
import sys

# ---- pass/fail thresholds (change here) ---------------------------------
LOCAL_POS_ABS_M = 0.30        # local position error at a segment end: max(this, 5% of path so far)
LOCAL_POS_FRAC = 0.05
LOCAL_YAW_DEG = 5.0
GLOBAL_POS_M = 6.0            # ~2 sigma of the 3 m GPS noise
GLOBAL_YAW_DEG = 10.0
STILL_POS_M = 0.30            # drift allowed during a "wait" segment
STILL_YAW_DEG = 2.0


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def parse(spec):
    """[(label, duration_s, v, w_deg_s)]"""
    out = []
    for part in spec.split(';'):
        part = part.strip()
        if not part:
            continue
        kind, _, arg = part.partition(':')
        if kind == 'wait':
            out.append((part, float(arg), 0.0, 0.0))
        elif kind == 'drive':
            dist, _, speed = arg.partition('@')
            dist, speed = float(dist), abs(float(speed))
            out.append((part, abs(dist) / speed, math.copysign(speed, dist), 0.0))
        elif kind == 'turn':
            angle, _, rate = arg.partition('@')
            angle, rate = float(angle), abs(float(rate))
            out.append((part, abs(angle) / rate, 0.0, math.copysign(rate, angle)))
    return out


def expected_track(segments, hz=20):
    """Dense [(t, x, y, yaw_deg)] and the segment-end rows."""
    x = y = yaw = 0.0
    t = 0.0
    dense = [(0.0, 0.0, 0.0, 0.0)]
    ends = []
    dt = 1.0 / hz
    for label, duration, v, w in segments:
        steps = max(int(round(duration * hz)), 1)
        for _ in range(steps):
            yaw += w * dt
            x += v * math.cos(math.radians(yaw)) * dt
            y += v * math.sin(math.radians(yaw)) * dt
            t += dt
            dense.append((t, x, y, yaw))
        ends.append((label, t, x, y, yaw))
    return dense, ends


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def nearest(rows, t, key='t_abs'):
    lo, hi = 0, len(rows) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if float(rows[mid][key]) < t:
            lo = mid + 1
        else:
            hi = mid
    return rows[lo]


def f(v):
    return float(v) if v not in ('', None) else None


def main(run):
    spec = [l.split(': ', 1)[1].strip() for l in open(os.path.join(run, 'path.txt'))
            if l.startswith('spec:')][0]
    name = [l.split(': ', 1)[1].strip() for l in open(os.path.join(run, 'path.txt'))
            if l.startswith('name:')][0]
    segments = parse(spec)
    dense, ends = expected_track(segments)
    truth = load(os.path.join(run, 'truth.csv'))
    gps = load(os.path.join(run, 'gps.csv'))
    est = [r for r in load(os.path.join(run, 'estimates.csv')) if r['local_x']]
    t0 = float(truth[0]['t_abs'])
    e0 = (float(gps[0]['meas_east_m']) - float(gps[0]['true_east_m']),
          float(gps[0]['meas_north_m']) - float(gps[0]['true_north_m']))

    def at(t):  # estimate row closest to path time t
        return nearest(est, t0 + t)

    checks = []

    def check(name_, ok, detail):
        checks.append((name_, ok, detail))

    rows = []
    out = ['EXPECTED vs ACTUAL   path "%s"   run %s' % (name, os.path.basename(run.rstrip('/'))),
           'path: ' + spec,
           'first GPS fix error (map-origin offset removed for the global EKF): E=%.2f N=%.2f m' % e0, '',
           'PER-SEGMENT (state at the END of each segment; position m, yaw deg)',
           '%-22s %6s | %-22s | %-22s | %-22s' % (
               'segment', 't_end', 'EXPECTED x,y,yaw', 'LOCAL EKF x,y,yaw (err)', 'GLOBAL EKF x,y,yaw (err)')]
    path_len = 0.0
    prev = (0.0, 0.0)
    prev_end = None
    for label, t_end, ex, ey, eyaw in ends:
        path_len += math.hypot(ex - prev[0], ey - prev[1])
        prev = (ex, ey)
        r = at(t_end - 0.05)
        lx, ly, lyaw = f(r['local_x']), f(r['local_y']), f(r['local_yaw_deg'])
        lerr = math.hypot(lx - ex, ly - ey)
        lyerr = abs(wrap(lyaw - eyaw))
        gx = f(r['global_x'])
        gtxt, gerr, gyerr = '(no data)', None, None
        if gx is not None:
            gy, gyaw = f(r['global_y']), f(r['global_yaw_deg'])
            gerr = math.hypot(gx - (ex - e0[0]), gy - (ey - e0[1]))
            gyerr = abs(wrap(gyaw - eyaw))
            gtxt = '%6.2f,%6.2f,%6.1f (%.1fm %.0f°)' % (gx, gy, gyaw, gerr, gyerr)
        out.append('%-22s %5.0fs | %6.2f,%6.2f,%7.1f | %6.2f,%6.2f,%6.1f (%.2fm %.1f°) | %s' % (
            label, t_end, ex, ey, eyaw, lx, ly, lyaw, lerr, lyerr, gtxt))
        rows.append([label, '%.1f' % t_end, ex, ey, eyaw, lx, ly, lyaw, lerr, lyerr, gx, gerr, gyerr])
        limit = max(LOCAL_POS_ABS_M, LOCAL_POS_FRAC * path_len)
        check('local position at end of "%s"' % label, lerr <= limit, '%.2f m (limit %.2f)' % (lerr, limit))
        check('local yaw at end of "%s"' % label, lyerr <= LOCAL_YAW_DEG, '%.1f° (limit %.0f)' % (lyerr, LOCAL_YAW_DEG))
        if gerr is not None:
            check('global position at end of "%s"' % label, gerr <= GLOBAL_POS_M,
                  '%.2f m (limit %.0f)' % (gerr, GLOBAL_POS_M))
            check('global yaw at end of "%s"' % label, gyerr <= GLOBAL_YAW_DEG,
                  '%.1f° (limit %.0f)' % (gyerr, GLOBAL_YAW_DEG))
        if label.startswith('wait') and prev_end is not None:
            a = at(prev_end + 0.5)
            b = at(t_end - 0.05)
            drift = math.hypot(f(b['local_x']) - f(a['local_x']), f(b['local_y']) - f(a['local_y']))
            ydrift = abs(wrap(f(b['local_yaw_deg']) - f(a['local_yaw_deg'])))
            check('local EKF holds still during "%s"' % label,
                  drift <= STILL_POS_M and ydrift <= STILL_YAW_DEG,
                  'moved %.2f m, yaw changed %.1f° (limits %.1f m, %.0f°)' % (drift, ydrift, STILL_POS_M, STILL_YAW_DEG))
        prev_end = t_end

    # ---- other checks -----------------------------------------------------
    diag_path = os.path.join(run, 'diagnostics.txt')
    bad = [l for l in open(diag_path)] if os.path.exists(diag_path) else []
    errors = [l.strip() for l in bad if l.startswith('ERROR')]
    check('no ERROR-level diagnostics', not errors, '; '.join(errors)[:200] or 'none')
    rates_path = os.path.join(run, 'topic_rates.txt')
    missing = []
    if os.path.exists(rates_path):
        seen = {l.split()[0] for l in open(rates_path) if l.strip()}
        missing = [k for k in ('gps_validated', 'zed_odom_base', 'wheel_odom', 'zed_imu', 'icm_imu', 'heading')
                   if k not in seen]
    check('all validated inputs reached the EKFs', not missing, ('missing: ' + ', '.join(missing)) if missing else 'yes')

    # ---- plots --------------------------------------------------------------
    plot_note = ''
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        ts = [t0 + d[0] for d in dense]
        series = {k: [] for k in ('lx', 'ly', 'lyaw', 'gx', 'gy', 'gyaw', 'px', 'py', 'tt')}
        for r in est:
            t = f(r['t_abs']) - t0
            if t < 0 or t > dense[-1][0] + 1:
                continue
            series['tt'].append(t)
            series['lx'].append(f(r['local_x'])); series['ly'].append(f(r['local_y']))
            series['lyaw'].append(f(r['local_yaw_deg']))
            series['gx'].append(f(r['global_x'])); series['gy'].append(f(r['global_y']))
            series['gyaw'].append(f(r['global_yaw_deg']))
            series['px'].append(f(r['gpsodom_x'])); series['py'].append(f(r['gpsodom_y']))
        have_g = any(v is not None for v in series['gx'])
        fig, ax = plt.subplots(1, 3, figsize=(18, 5.5))
        ax[0].plot([d[1] for d in dense], [d[2] for d in dense], 'k-', lw=3, label='EXPECTED')
        ax[0].plot(series['lx'], series['ly'], 'b-', label='local EKF (odom)')
        if have_g:
            ax[0].plot([d[1] - e0[0] for d in dense], [d[2] - e0[1] for d in dense], 'k:', lw=1.5,
                       label='expected in map frame')
            ax[0].plot([v for v in series['gx'] if v is not None], [v for v in series['gy'] if v is not None],
                       'r-', label='global EKF (map)')
            ax[0].plot([v for v in series['px'] if v is not None], [v for v in series['py'] if v is not None],
                       '.', color='orange', ms=2, label='raw GPS')
        ax[0].set_aspect('equal'); ax[0].set_title('Path (x east, y north, m)'); ax[0].legend(); ax[0].grid(True)
        ax[1].plot([d[0] for d in dense], [wrap(d[3]) for d in dense], 'k-', lw=3, label='EXPECTED')
        ax[1].plot(series['tt'], [wrap(v) for v in series['lyaw']], 'b-', label='local EKF')
        if have_g:
            ax[1].plot([t for t, v in zip(series['tt'], series['gyaw']) if v is not None],
                       [wrap(v) for v in series['gyaw'] if v is not None], 'r-', label='global EKF')
        ax[1].set_title('Yaw (deg)'); ax[1].set_xlabel('s'); ax[1].legend(); ax[1].grid(True)

        def exp_at(t):
            i = min(max(int(round(t * 20)), 0), len(dense) - 1)
            return dense[i]
        le = [math.hypot(x - exp_at(t)[1], y - exp_at(t)[2])
              for t, x, y in zip(series['tt'], series['lx'], series['ly'])]
        ax[2].plot(series['tt'], le, 'b-', label='local EKF error')
        if have_g:
            ge = [math.hypot(x - (exp_at(t)[1] - e0[0]), y - (exp_at(t)[2] - e0[1]))
                  for t, x, y in zip(series['tt'], series['gx'], series['gy']) if x is not None]
            gt = [t for t, x in zip(series['tt'], series['gx']) if x is not None]
            ax[2].plot(gt, ge, 'r-', label='global EKF error')
        ax[2].set_title('Position error vs expected (m)'); ax[2].set_xlabel('s'); ax[2].legend(); ax[2].grid(True)
        fig.tight_layout()
        fig.savefig(os.path.join(run, 'path_compare.png'), dpi=90)
        plot_note = 'plot: path_compare.png'
    except Exception as error:  # noqa: BLE001
        plot_note = 'plot skipped: %s' % error

    out += ['', 'CHECKS']
    for name_, ok, detail in checks:
        out.append('  %s  %s: %s' % ('PASS' if ok else 'FAIL', name_, detail))
    failed = [c for c in checks if not c[1]]
    out += ['', 'RESULT: %s  (%d/%d checks passed)' % ('ALL PASS' if not failed else 'FAILURES', len(checks) - len(failed), len(checks))]
    out += ['', 'TOPIC RATES (validated stream into the EKFs)',
            open(rates_path).read().rstrip() if os.path.exists(rates_path) else '  (missing)',
            '', 'DIAGNOSTICS (last seen)',
            ''.join(bad).rstrip() if bad else '  (missing)']
    log = os.path.join(run, 'localization.log')
    if os.path.exists(log):
        text = re.sub(r'\x1b\[[0-9;]*m', '', open(log, errors='replace').read())
        warns = [l for l in text.splitlines() if 'WARN' in l or 'ERROR' in l]
        out += ['', 'LOG WARNINGS/ERRORS in localization.log: %d' % len(warns)]
        seen = {}
        for l in warns:
            key = re.sub(r'\[\d+\.\d+\]', '', l.split('] ', 1)[-1])[:150]
            seen[key] = seen.get(key, 0) + 1
        out += ['  x%-4d %s' % (n, k) for k, n in sorted(seen.items(), key=lambda kv: -kv[1])[:10]]
    out += ['', plot_note, 'files: summary.txt segments.csv estimates.csv truth.csv gps.csv '
            'localization.log fake_sensors.log recorder.log']
    text = '\n'.join(out)
    open(os.path.join(run, 'summary.txt'), 'w').write(text + '\n')
    with open(os.path.join(run, 'segments.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['segment', 't_end', 'exp_x', 'exp_y', 'exp_yaw', 'local_x', 'local_y', 'local_yaw',
                    'local_pos_err', 'local_yaw_err', 'global_x', 'global_pos_err', 'global_yaw_err'])
        w.writerows(rows)
    print(text)


if __name__ == '__main__':
    main(sys.argv[1])
