#!/usr/bin/env python3

import argparse
import os
import runpy
import sys

from ament_index_python.packages import get_package_prefix
from interbotix_common_modules.common_robot.robot import robot_shutdown
import interbotix_xs_modules.xs_robot.arm as interbotix_arm
import xsarm_scripts.supervisor as supervisor_package
from xsarm_scripts.supervisor import MotionRejected, SupervisedManipulator

"""
Run an unmodified Interbotix demo with the supervisor in charge of the arm (task 2.10).

The demo's `InterbotixManipulatorXS` is replaced by `SupervisedManipulator`, so every move it
asks for is checked; a refused move ends the demo with its reason, before anything of that
move is sent, and the arm is then parked (home, then sleep). A demo that reaches under the
API (`bot.core`: trajectories, currents, PWM) is stopped at that point, before it sends
anything.

    ros2 run xsarm_scripts supervised_demo bartender
    ros2 run xsarm_scripts supervised_demo ee_cartesian_trajectory --speed 1.0

A name is looked up among interbotix_xsarm_control's installed demos; a path is run as it is.
The demo runs on the vx300s whatever model it names (the installed copies name the arm they
were written for, such as the wx250). The last
line printed is the outcome: `DEMO <name> completed`, `refused <code> <detail>` or
`blocked <reason>`. Exit status 0 for any of those, 1 if the demo failed otherwise.
"""


def demo_path(name):
    if os.path.exists(name):
        return name
    # installed as executables of the package, not under share/
    return os.path.join(get_package_prefix('interbotix_xsarm_control'), 'lib',
                        'interbotix_xsarm_control', f'{name}.py')


def main():
    parser = argparse.ArgumentParser(description='Run an Interbotix demo under the supervisor.')
    parser.add_argument('demo', help="demo name, such as 'bartender', or a path")
    parser.add_argument('--speed', type=float, default=None,
                        help='joint speed limit [rad/s] for this run, below the usual 2.5')
    parser.add_argument('--robot-model', default='vx300s',
                        help='the arm, whatever model the demo was written for')
    args = parser.parse_args()
    path = demo_path(args.demo)
    name = os.path.splitext(os.path.basename(path))[0]
    bots = []

    def supervised(robot_model=None, group_name='arm', gripper_name='gripper', **kwargs):
        # the installed demos name the model they were written for (wx250 and others); the
        # arm here is a vx300s, and the supervisor knows no other
        bots.append(SupervisedManipulator(
            robot_model=args.robot_model, group_name=group_name, gripper_name=gripper_name,
            speed_limit=args.speed, **kwargs))
        return bots[-1]

    interbotix_arm.InterbotixManipulatorXS = supervised
    if args.speed is not None:
        # a script that is supervised already (such as the lab template) gets the speed too
        def at_speed(*positional, **kwargs):
            kwargs.setdefault('speed_limit', args.speed)
            bots.append(SupervisedManipulator(*positional, **kwargs))
            return bots[-1]
        supervisor_package.SupervisedManipulator = at_speed
    sys.argv = [path]
    outcome, status = f'DEMO {name} completed', 0
    try:
        runpy.run_path(path, run_name='__main__')
    except MotionRejected as rejection:
        outcome = f'DEMO {name} refused {rejection.code} {rejection.detail}'
    except AttributeError as problem:
        if 'not available through the arm supervisor' not in str(problem):
            raise
        outcome = f'DEMO {name} blocked {problem}'
    except SystemExit as leaving:
        if leaving.code not in (None, 0):
            outcome, status = f'DEMO {name} failed: exit status {leaving.code}', 1
    if not outcome.endswith('completed') and bots:
        try:
            bots[-1].arm.go_to_home_pose()
            bots[-1].arm.go_to_sleep_pose()
        except MotionRejected as rejection:
            print(f'Could not park the arm: {rejection}')
        try:
            robot_shutdown()
        except Exception:   # the demo may have shut the node down itself
            pass
    print(outcome, flush=True)
    sys.exit(status)


if __name__ == '__main__':
    main()
