"""The repository check command must fail honestly and locate build inputs."""
import importlib.util
import contextlib
import io
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

PATH = Path(__file__).resolve().parents[3] / 'scripts' / 'check_localization.py'


class VerificationCommandTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('check_localization', PATH)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_command_failure_is_recorded_and_returns_false(self):
        results = []
        with contextlib.redirect_stdout(io.StringIO()), patch.object(
                self.module.subprocess, 'run', return_value=subprocess.CompletedProcess([], 2)):
            self.assertFalse(self.module.execute('failure', ['false'], results))
        self.assertEqual(results[0]['exit_code'], 2)

    def test_command_timeout_is_a_failure(self):
        results = []
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()), patch.object(
                self.module.subprocess, 'run', side_effect=subprocess.TimeoutExpired('test', 1)):
            self.assertFalse(self.module.execute('timeout', ['test'], results))
        self.assertEqual(results[0]['exit_code'], 124)

    def test_explicit_eigen_path_is_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            with self.assertRaisesRegex(RuntimeError, 'Eigen'):
                self.module.find_eigen(path)
            (path / 'Eigen').mkdir()
            (path / 'Eigen/Core').write_text('// test header')
            self.assertEqual(self.module.find_eigen(path), path)


if __name__ == '__main__':
    unittest.main()
