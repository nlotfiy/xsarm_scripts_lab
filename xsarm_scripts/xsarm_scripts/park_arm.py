#!/usr/bin/env python3

import argparse
import math
import sys
import time

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from xsarm_scripts.supervisor import MotionRejected, SupervisedManipulator
from xsarm_scripts.supervisor.envelope import load_envelope

"""
Put the arm in its sleep pose: home first, then sleep, through the supervisor.

    ros2 run xsarm_scripts park_arm

`driver_guard` runs this when the driver is asked to stop, because the driver switches the
motors' torque off as it exits and an arm that is not in its sleep pose then drops. It first
waits for the arm to come to rest, so that a tool that is parking the arm on its own way out
is left to finish. Exit status: 0 if the arm is in its sleep pose, 1 if it is not.
"""

# joint positions [rad] within which the arm counts as being in its sleep pose
AT_REST = 0.1
# the arm counts as still when no joint has moved more than this [rad] over `STILL_FOR` [s]
STILL, STILL_FOR, STILL_TIMEOUT = 0.01, 0.6, 15.0


def wait_until_still(arm):
    """Wait for the measured arm to stop moving; return whether it did."""
    deadline = time.monotonic() + STILL_TIMEOUT
    while time.monotonic() < deadline:
        before = arm.get_joint_positions()
        time.sleep(STILL_FOR)
        after = arm.get_joint_positions()
        if max(abs(a - b) for a, b in zip(after, before)) <= STILL:
            return True
    return False


def park(bot, log=print):
    """
    Bring the arm to its sleep pose if it is not there.

    :return: `True` if the arm is in its sleep pose afterwards
    """
    arm = bot.arm
    sleep = arm.group_info.joint_sleep_positions
    if not all(math.isfinite(q) for q in arm.get_joint_positions()):
        log('The arm\'s position cannot be read (the driver reports no valid joint states); '
            'not parking it.')
        return False
    if not wait_until_still(arm):
        log('The arm is still being moved by something else; not parking it.')
        return False
    if max(abs(q - s) for q, s in zip(arm.get_joint_positions(), sleep)) <= AT_REST:
        log('The arm is in its sleep pose.')
        return True
    arm.capture_joint_positions()   # whatever moved it last, start from where it is
    try:
        log('Parking the arm: home, then sleep.')
        arm.go_to_home_pose()
        arm.go_to_sleep_pose()
    except MotionRejected as rejection:
        log(f'The arm could not be parked: {rejection}')
        return False
    log('The arm is parked.')
    return True


def main():
    parser = argparse.ArgumentParser(description='Put the arm in its sleep pose.')
    parser.add_argument('--robot-model', default='vx300s')
    parser.add_argument('--robot-name', default=None)
    parser.add_argument('--local-envelope', action='store_true',
                        help="use this computer's envelope file instead of waiting for the "
                             "one the arm's computer publishes (for driver_guard, which runs "
                             'on the arm\'s computer as envelope_server stops)')
    args = parser.parse_args()
    envelope = {'work_envelope': load_envelope()} if args.local_envelope else {}
    bot = SupervisedManipulator(
        robot_model=args.robot_model, robot_name=args.robot_name, **envelope)
    robot_startup()
    try:
        parked = park(bot)
    finally:
        robot_shutdown()
    sys.exit(0 if parked else 1)


if __name__ == '__main__':
    main()
