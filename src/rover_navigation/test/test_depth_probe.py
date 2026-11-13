"""Regression: sky pixels precede valid ground pixels in depth clouds."""
import importlib.util
from pathlib import Path

from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

path = Path(__file__).resolve().parents[3] / 'scripts' / 'probe_simulation.py'
spec = importlib.util.spec_from_file_location('simulation_probe_under_test', path)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def test_infinite_sky_does_not_hide_finite_ground():
    cloud = create_cloud_xyz32(Header(), [(float('inf'),)*3, (1., 2., 3.)])
    assert probe.has_finite_depth(cloud)


def test_all_infinite_or_nan_cloud_is_invalid():
    cloud = create_cloud_xyz32(Header(), [(float('inf'),)*3, (float('nan'),)*3])
    assert not probe.has_finite_depth(cloud)
