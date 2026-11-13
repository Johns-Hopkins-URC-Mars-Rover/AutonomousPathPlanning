"""Run real Gazebo motion then Nav2 checks, cleaning each launch process tree."""
import os
from pathlib import Path
import signal
import subprocess
import sys


def run_profile(name, probe_option, nav2):
    path = Path('/tmp') / ('simulation-' + name + '.log')
    with path.open('w') as stream:
        process = subprocess.Popen([
            'ros2', 'launch', 'rover_description', 'simulation.launch.py',
            'enable_nav2:=' + nav2,
        ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            # Probe owns bounded startup/motion/action deadlines. A child ROS
            # crash yields a failed probe rather than a successful launch-only test.
            subprocess.run([
                sys.executable, '/source/scripts/probe_simulation.py', probe_option,
            ], check=True, timeout=480)
        except BaseException:
            print(path.read_text()[-12000:], flush=True)
            raise
        finally:
            if process.poll() is None:
                # Let ROS launch shut down its children once. Signaling the
                # entire group here also hits children directly a second time.
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            # Launch may leave children after its own early exit.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    print('PASS simulation profile: ' + name, flush=True)


if __name__ == '__main__':
    run_profile('motion', '--motion', 'false')
    run_profile('navigation', '--navigation', 'true')
