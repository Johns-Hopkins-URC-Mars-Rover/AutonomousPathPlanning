"""Seeded synthetic errors; these magnitudes are stress assumptions, not calibration."""
import math
import random
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from sensor_contracts import diagonal, multiply, rotate


def euler_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class NoiseModel:
    position_sigma = .035  # m, white noise per world axis
    angle_sigma = (.006, .006, .015)  # rad, world-axis small rotation
    gyro_sigma = .006  # rad/s per sensor axis
    camera_mount_position = (.20, 0., .10)  # Known synthetic mount, m
    camera_mount_orientation = euler_quaternion(0., 0., .35)
    imu_mount_orientation = euler_quaternion(.15, 0., .40)

    def __init__(self, seed, profile):
        if profile not in ('noisy', 'tracking_fault'):
            raise ValueError('unknown fixture profile')
        self.profile = profile
        self.camera_rng = random.Random(seed)
        self.imu_rng = random.Random(seed+1)
        self.delay_rng = random.Random(seed+2)
        self.camera_walk = [0., 0., 0.]
        self.gyro_walk = [0., 0., 0.]
        self.last_camera = self.last_imu = None
        self.outlier_sent = False

    @staticmethod
    def truth(t):
        x = .15*min(2., max(0., t-2.))-.15*min(2., max(0., t-8.))
        yaw = math.pi/8*min(4., max(0., t-4.))
        rate = math.pi/8 if 4. <= t < 8. else 0.
        return (x, 0., 0.), euler_quaternion(0., 0., yaw), (0., 0., rate)

    def camera(self, t):
        p, q, _ = self.truth(t)
        dt = max(0., t-self.last_camera) if self.last_camera is not None else 1./30.
        self.last_camera = t
        for i in range(3):
            self.camera_walk[i] += self.camera_rng.gauss(0., .0015)*math.sqrt(dt)
        lever = rotate(q, self.camera_mount_position)
        position = [p[i]+lever[i]+self.camera_walk[i]+self.camera_rng.gauss(0., self.position_sigma)
                    for i in range(3)]
        position[2] += .004*t  # Slow unmodelled VO vertical drift, m/s.
        if t >= 4.2 and not self.outlier_sent:
            position = [v+offset for v, offset in zip(position, (.25, -.15, .35))]
            self.outlier_sent = True
        if self.profile == 'tracking_fault' and 5. <= t < 7.:
            position = [offset+self.camera_rng.gauss(0., self.position_sigma)
                        for offset in (-7.1128, .1515, -15.4141)]
        perturbation = euler_quaternion(*(self.camera_rng.gauss(0., s) for s in self.angle_sigma))
        orientation = multiply(perturbation, multiply(q, self.camera_mount_orientation))
        return dict(position=tuple(position), orientation=orientation,
                    covariance=diagonal([self.position_sigma**2]*3+[s*s for s in self.angle_sigma]))

    def imu(self, t):
        _, _, w = self.truth(t)
        dt = max(0., t-self.last_imu) if self.last_imu is not None else .01
        self.last_imu = t
        for i in range(3):
            self.gyro_walk[i] += self.imu_rng.gauss(0., .0002)*math.sqrt(dt)
        q = self.imu_mount_orientation
        sensor_rate = rotate((-q[0], -q[1], -q[2], q[3]), w)
        rate = tuple(sensor_rate[i]+bias+self.gyro_walk[i]+self.imu_rng.gauss(0., self.gyro_sigma)
                     for i, bias in enumerate((.002, -.001, .0015)))
        return dict(omega=rate, covariance=diagonal([self.gyro_sigma**2]*3))

    def latency(self, sensor):
        if sensor not in ('camera', 'imu'):
            raise ValueError('unknown sensor')
        return self.delay_rng.uniform(0., .08 if sensor == 'camera' else .015)
