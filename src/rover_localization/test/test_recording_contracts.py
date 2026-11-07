import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[3] / 'scripts' / 'run_imu_test.py'
spec = importlib.util.spec_from_file_location('run_imu_test', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RecordingContractsTest(unittest.TestCase):
    def test_records_camera_gps_and_localization_evidence(self):
        for topic in ('/zed/zed_node/odom', '/odometry/local', '/odometry/global',
                      '/gps/fix', '/diagnostics', '/tf', '/tf_static'):
            self.assertIn(topic, module.BAG_TOPICS)

    def test_zed_only_run_does_not_require_icm(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'samples.csv'
            p.write_text('source,stamp\nzed,1\nfused,1\n')
            self.assertTrue(module.check_csv(p, require_icm=False)[0])
            self.assertFalse(module.check_csv(p, require_icm=True)[0])


if __name__ == '__main__':
    unittest.main()
