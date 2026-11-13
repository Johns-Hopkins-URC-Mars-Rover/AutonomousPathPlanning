"""Startup GPS error calibration: measure real fix scatter, set the driver's variance.

Keep the rover parked while this runs (default 60 s after the first fix). It
logs live progress, then sets ``position_variance`` on the running GPS driver
and writes the result to a file. If anything goes wrong the driver keeps the
conservative value from gps_params.yaml.
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .gps_error import estimate_variance, to_local_en


class GpsErrorCalibrator(Node):
    def __init__(self):
        super().__init__('gps_error_calibrator')
        self.duration = self.declare_parameter('duration_s', 60.0).value
        self.inflation = self.declare_parameter('inflation', 2.0).value
        self.min_var = self.declare_parameter('min_variance', 4.0).value
        self.max_var = self.declare_parameter('max_variance', 100.0).value
        self.max_drift = self.declare_parameter('max_drift_m', 15.0).value
        self.max_retries = self.declare_parameter('max_retries', 3).value
        self.fix_timeout = self.declare_parameter('fix_timeout_s', 120.0).value
        self.target = self.declare_parameter('target_node', 'l76x_gps_publisher').value
        self.out_file = os.path.expanduser(self.declare_parameter(
            'output_file', '~/.ros/gps_error_calibration.yaml').value)

        self.samples = []       # (lat, lon)
        self.alts = []
        self.window_start = None
        self.started = time.monotonic()
        self.retries = 0
        self.done = False
        self.client = self.create_client(SetParameters, '/%s/set_parameters' % self.target)
        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)
        self.get_logger().info(
            'CALIBRATING GPS error: keep the rover parked for about %.0f s after the first '
            'fix. The driver uses its configured (conservative) variance until this finishes.'
            % self.duration)

    def on_fix(self, msg):
        if (self.done or msg.status.status < NavSatStatus.STATUS_FIX
                or not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude))):
            return
        if self.window_start is None:
            self.window_start = time.monotonic()
        self.samples.append((msg.latitude, msg.longitude))
        self.alts.append(msg.altitude if math.isfinite(msg.altitude) else None)

    def tick(self):
        if self.done:
            return
        now = time.monotonic()
        if self.window_start is None:
            waited = now - self.started
            if waited > self.fix_timeout:
                return self.finish('no GPS fix after %.0f s' % waited, None)
            self.get_logger().info('CALIBRATING GPS error: waiting for a fix (%.0f s)...' % waited,
                                   throttle_duration_sec=5.0)
            return
        elapsed = now - self.window_start
        if elapsed < self.duration:
            en = to_local_en(self.samples)
            n = len(en)
            if n >= 3:
                mean_e = sum(p[0] for p in en) / n
                mean_n = sum(p[1] for p in en) / n
                sd_e = math.sqrt(sum((p[0] - mean_e) ** 2 for p in en) / (n - 1))
                sd_n = math.sqrt(sum((p[1] - mean_n) ** 2 for p in en) / (n - 1))
                self.get_logger().info(
                    'CALIBRATING GPS error %3d%% | ~%.0f s left | %d fixes | '
                    'spread so far E=%.1f m N=%.1f m'
                    % (100 * elapsed / self.duration, self.duration - elapsed, n, sd_e, sd_n))
            return
        self.evaluate()

    def evaluate(self):
        en = to_local_en(self.samples)
        up = None
        if all(a is not None for a in self.alts):
            up = self.alts
        result = estimate_variance(en, up, self.inflation, self.min_var,
                                   self.max_var, self.max_drift)
        if result['ok']:
            return self.finish(None, result)
        self.retries += 1
        if self.retries >= self.max_retries:
            return self.finish('%s; gave up after %d tries' % (result['reason'], self.retries), result)
        self.get_logger().warn('CALIBRATING GPS error: %s. Restart %d/%d, hold still for %.0f s.'
                               % (result['reason'], self.retries, self.max_retries, self.duration))
        self.samples, self.alts, self.window_start = [], [], None
        self.started = time.monotonic()

    def finish(self, problem, result):
        self.done = True
        if problem:
            self.get_logger().error(
                'GPS ERROR CALIBRATION FAILED: %s. Keeping the configured position_variance.' % problem)
            return self.shutdown()
        var = result['variance']
        self.get_logger().info(
            'GPS ERROR CALIBRATION DONE: %d fixes, spread E=%.1f m N=%.1f m, drift %.1f m. '
            'Variance set to [%.1f, %.1f, %.1f] m^2 (horizontal std %.1f m, includes x%.1f margin).'
            % (result['n'], result['std_e'], result['std_n'], result['drift_m'],
               var[0], var[1], var[2], math.sqrt(var[0]), self.inflation))
        self.save(result)
        self.push(var)

    def push(self, var):
        if not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('GPS driver parameter service not found; variance NOT applied.')
            return self.shutdown()
        param = Parameter('position_variance', Parameter.Type.DOUBLE_ARRAY, [float(v) for v in var])
        request = SetParameters.Request(parameters=[param.to_parameter_msg()])
        self.client.call_async(request).add_done_callback(self.pushed)

    def pushed(self, future):
        try:
            reason = future.result().results[0].reason
            ok = future.result().results[0].successful
        except Exception as error:  # noqa: BLE001
            ok, reason = False, str(error)
        if ok:
            self.get_logger().info('Applied to the GPS driver. Calibration complete.')
        else:
            self.get_logger().error('GPS driver rejected the variance: %s' % reason)
        self.shutdown()

    def save(self, result):
        try:
            os.makedirs(os.path.dirname(self.out_file), exist_ok=True)
            with open(self.out_file, 'w') as f:
                f.write('# Written by gps_error_calibrator at startup.\n')
                f.write('time: %s\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
                f.write('fixes: %d\nstd_east_m: %.3f\nstd_north_m: %.3f\ndrift_m: %.3f\n'
                        % (result['n'], result['std_e'], result['std_n'], result['drift_m']))
                f.write('position_variance: [%.3f, %.3f, %.3f]\n' % tuple(result['variance']))
        except OSError as error:
            self.get_logger().warn('Could not save calibration file: %s' % error)

    def shutdown(self):
        # Let the log flush, then exit cleanly so the launch carries on.
        self.create_timer(0.2, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    node = GpsErrorCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
