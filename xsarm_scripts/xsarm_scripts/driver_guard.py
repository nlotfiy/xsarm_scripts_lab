#!/usr/bin/env python3

import argparse
import signal
import subprocess
import sys
import threading

"""
Run the arm's driver so that Ctrl+C parks the arm before the driver stops.

    driver_guard [options] -- <the driver's command line>

The driver (`xs_sdk`) switches the torque of every motor off as it exits, so stopping it
while the arm is up lets the arm drop. This program starts the driver in a session of its
own, where the terminal's Ctrl+C does not reach it, and when it is itself told to stop:

1. it runs `park_arm`, which takes the arm home and then to its sleep pose;
2. if that succeeds, it stops the driver;
3. if it does not (the supervisor refused the move, or the arm cannot be read), it leaves the
   driver running, with the arm holding its position, and says so. Ctrl+C a second time
   stops the driver regardless: support the arm by hand first, because it will drop.

It is put in front of the driver by `launch/arm.launch.py`.
"""

BANNER = '=' * 78


def say(text):
    print(f'\n{BANNER}\n[driver_guard] {text}\n{BANNER}', flush=True)


class Guard:
    """The driver process, and what to do when asked to stop it."""

    def __init__(self, driver_command, park_command, park_timeout, stop_timeout=10.0):
        self.park_command, self.park_timeout = park_command, park_timeout
        self.stop_timeout = stop_timeout
        self.state = 'running'   # -> 'parking' -> 'stopping', or 'holding' -> 'stopping'
        self.lock = threading.Lock()
        # a session of its own: a Ctrl+C typed at the terminal goes to this program only
        self.driver = subprocess.Popen(driver_command, start_new_session=True)

    def on_signal(self, signum, frame):
        with self.lock:
            state = self.state
            if state == 'running':
                self.state = 'parking'
                threading.Thread(target=self.park_then_stop, daemon=True).start()
            elif state == 'holding' and signum == signal.SIGINT:
                self.state = 'stopping'
                threading.Thread(target=self.stop_driver, daemon=True).start()
        if state == 'parking':
            print('[driver_guard] parking the arm, please wait...', flush=True)
        elif state == 'holding' and signum == signal.SIGINT:
            say('Stopping the driver without parking: THE ARM WILL DROP.')

    def park_then_stop(self):
        say('Stop requested. Parking the arm before the driver stops; keep clear.')
        try:
            parked = subprocess.run(
                self.park_command, start_new_session=True,
                timeout=self.park_timeout).returncode == 0
        except (subprocess.TimeoutExpired, OSError) as problem:
            print(f'[driver_guard] parking did not finish: {problem}', flush=True)
            parked = False
        with self.lock:
            self.state = 'stopping' if parked else 'holding'
        if parked:
            self.stop_driver()
        else:
            say('THE ARM IS NOT PARKED. The driver is still running and the arm is holding '
                'its position.\n[driver_guard] Support the arm by hand, then press Ctrl+C '
                'again to stop the driver (the arm will drop).')

    def stop_driver(self):
        self.driver.send_signal(signal.SIGINT)
        try:
            self.driver.wait(self.stop_timeout)
        except subprocess.TimeoutExpired:
            self.driver.terminate()

    def run(self):
        signal.signal(signal.SIGINT, self.on_signal)
        signal.signal(signal.SIGTERM, self.on_signal)
        while True:
            try:
                return self.driver.wait(0.2)
            except subprocess.TimeoutExpired:
                continue


def main():
    arguments = sys.argv[1:]
    if '--' not in arguments:
        sys.exit('usage: driver_guard [options] -- <driver command line>')
    split = arguments.index('--')
    parser = argparse.ArgumentParser(
        description='Run the driver so that Ctrl+C parks the arm before the driver stops.')
    parser.add_argument('--robot-model', default='vx300s')
    parser.add_argument('--robot-name', default=None)
    parser.add_argument('--park-timeout', type=float, default=40.0,
                        help='time [s] the arm is given to reach its sleep pose')
    parser.add_argument('--park-command', nargs='+', default=None,
                        help='command that parks the arm (default: park_arm)')
    args = parser.parse_args(arguments[:split])
    park_command = args.park_command or [
        sys.executable, '-m', 'xsarm_scripts.park_arm', '--local-envelope',
        '--robot-model', args.robot_model,
        *(['--robot-name', args.robot_name] if args.robot_name else [])]
    sys.exit(Guard(arguments[split + 1:], park_command, args.park_timeout).run())


if __name__ == '__main__':
    main()
