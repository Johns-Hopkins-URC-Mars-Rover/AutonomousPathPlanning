"""NMEA and startup regressions without ROS, serial or Jetson imports."""
import ast
import importlib.util
import math
from pathlib import Path
import types
import unittest

ROOT = Path(__file__).resolve().parents[1] / 'l76x_gps_interface'


def functions(path):
    tree = ast.parse(path.read_text())
    tree.body = [n for n in tree.body if isinstance(n, ast.FunctionDef)]
    namespace = {'math': math}
    exec(compile(tree, str(path), 'exec'), namespace)
    return namespace


def sentence(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f'${body}*{checksum:02X}'.encode()


class GpsContractTest(unittest.TestCase):
    def source(self):
        path = ROOT / 'nmea.py'
        if not path.exists():
            return functions(ROOT / 'gps_publisher.py')
        spec = importlib.util.spec_from_file_location('nmea_under_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.__dict__

    def test_signed_decimal_coordinates(self):
        c = self.source()['decimal_degrees']
        self.assertAlmostEqual(c('3900.000', 'N', True), 39.)
        self.assertAlmostEqual(c('07630.000', 'W', False), -76.5)
        for v, hemisphere in (('9060.00', 'N'), ('nan', 'N'), ('1000', 'X')):
            with self.assertRaises(ValueError):
                c(v, hemisphere, True)

    def test_checksum_failure_and_no_fix(self):
        parse = self.source()['parse_rmc']
        self.assertIsNone(parse(b'$GPRMC,broken*00'))
        self.assertEqual(parse(sentence('GNRMC,120000,V,,,,,,,061126,,,N'))[2], 'V')

    def test_gga_provides_ellipsoid_altitude(self):
        c = self.source()
        self.assertIn('parse_gga', c, 'RMC-only output cannot provide altitude for navsat_transform')
        fields = c['parse_gga'](sentence('GPGGA,120000,3900.0,N,07600.0,W,1,08,1.0,50.0,M,-30.0,M,,'))
        self.assertEqual(c['ellipsoid_altitude'](fields), 20.)
        bad = list(fields)
        bad[6] = '0'
        with self.assertRaises(ValueError):
            c['ellipsoid_altitude'](bad)
        bad = list(fields)
        bad[11] = ''
        with self.assertRaises(ValueError):
            c['ellipsoid_altitude'](bad)

    def test_startup_failure_preserves_original_error(self):
        ns = functions(ROOT / 'gps_publisher.py')
        ns['rclpy'] = types.SimpleNamespace(init=lambda **kw: None, ok=lambda: False)
        def fail():
            raise RuntimeError('receiver unavailable')
        ns['GPSPublisher'] = fail
        with self.assertRaisesRegex(RuntimeError, 'receiver unavailable'):
            ns['main']()

    def test_receiver_utc_timestamp(self):
        c = self.source()
        fields = c['parse_rmc'](sentence('GPRMC,120000.25,A,3900.0,N,07600.0,W,0,0,061126,,,A'))
        from datetime import datetime, timezone
        expected = datetime(2026, 11, 6, 12, 0, tzinfo=timezone.utc).timestamp() + .25
        self.assertEqual(c['utc_timestamp'](fields), expected)
        fields[9] = ''
        with self.assertRaises(ValueError): c['utc_timestamp'](fields)

    def test_published_fix_has_source_time_and_only_configured_covariance(self):
        from types import SimpleNamespace as NS
        tree = ast.parse((ROOT / 'gps_publisher.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'publish_fix')
        ns = self.source()
        class Fix:
            COVARIANCE_TYPE_UNKNOWN, COVARIANCE_TYPE_DIAGONAL_KNOWN = 0, 2
            def __init__(self):
                self.header = NS(stamp=NS(), frame_id='')
                self.status = NS()
                self.position_covariance = [0.] * 9
        ns.update(NavSatFix=Fix, NavSatStatus=NS(STATUS_NO_FIX=-1, STATUS_FIX=0))
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<publish_fix>', 'exec'), ns)
        messages = []
        node = NS(frame_id='gps_link', variance=[0.,0.,0.], publisher=NS(publish=messages.append))
        fields = ns['parse_rmc'](sentence('GPRMC,120000,A,3900.0,N,07600.0,W,0,0,061126,,,A'))
        ns['publish_fix'](node, fields)
        self.assertTrue(math.isnan(messages[-1].altitude))
        self.assertEqual(messages[-1].position_covariance_type, 0)
        node.variance = [4., 4., 9.]
        gga = ns['parse_gga'](sentence('GPGGA,120000,3900.0,N,07600.0,W,1,08,1.0,50.0,M,-30.0,M,,'))
        ns['publish_fix'](node, fields, gga)
        self.assertEqual(messages[-1].altitude, 20.)
        self.assertEqual(messages[-1].position_covariance[8], 9.)
        self.assertEqual(messages[-1].header.stamp.sec, int(ns['utc_timestamp'](fields)))

    def test_invalid_nmea_and_altitude_inputs(self):
        c = self.source()
        for raw in (b'not-nmea', b'\xff', b'$GPRMC*0', b'$GPRMC*ZZ', sentence('OTHER,abc'),
                    sentence('GPRMC,120000,X,,,,,,,061126'), sentence('GPRMC')):
            self.assertIsNone(c['parse_rmc'](raw))
        self.assertIsNone(c['parse_gga'](sentence('GPGGA,short')))
        fields = c['parse_gga'](sentence('GPGGA,120000,3900.0,N,07600.0,W,1,08,1.0,nan,M,0,M,,'))
        with self.assertRaises(ValueError): c['ellipsoid_altitude'](fields)
        fields[9] = '2'; fields[10] = 'feet'
        with self.assertRaises(ValueError): c['ellipsoid_altitude'](fields)
        with self.assertRaises(ValueError): c['decimal_degrees']('18100', 'E', False)
        fields = c['parse_rmc'](sentence('GPRMC,120060,A,3900.0,N,07600.0,W,0,0,061126,,,A'))
        with self.assertRaises(ValueError): c['utc_timestamp'](fields)


if __name__ == '__main__':
    unittest.main()
