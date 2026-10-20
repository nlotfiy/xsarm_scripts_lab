#!/usr/bin/env python3

import argparse
import os
import sys
import time

from xsarm_scripts.supervisor.envelope import (
    AXES, checked_envelope, envelope_path, excess, grow_to_contain, load_envelope, rest_path,
    save_envelope)
from xsarm_scripts.supervisor.geometry import generate_urdf, RobotGeometry

"""
Set up the work envelope of the arm (procedure in docs/work_envelope.md).

The work envelope is a box in the arm's base frame. Run this on the computer the arm is plugged
into: once it is set, every supervised script that drives the arm, on this computer or another,
refuses any move that would take a link of the arm outside it.

    ros2 run xsarm_scripts work_envelope show
    ros2 run xsarm_scripts work_envelope set --min -0.30 -0.45 0.0 --max 0.60 0.45 0.70
    ros2 run xsarm_scripts work_envelope teach     # needs the driver running
    ros2 run xsarm_scripts work_envelope clear

`teach` switches the arm's torque off so that you can carry the gripper to the corners of the
space you want to allow. HOLD THE ARM before you release it: it drops when the torque goes off.
Two opposite corners are enough; any further points only make the box larger. `set` and
`teach` enlarge the box where needed so that it contains the arm at its home and sleep poses
and on the way between them, and say so.
"""

ROBOT_MODEL = 'vx300s'
JOINTS = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
SLEEP = [0.0, -1.85, 1.55, 0.0, 0.8, 0.0]
INTERACTIVE = sys.stdin.isatty()


def ask(prompt):
    """Wait for the operator, or carry straight on when there is no terminal."""
    if INTERACTIVE:
        return input(prompt).strip().lower()
    print(prompt)
    time.sleep(0.5)
    return ''


def describe(geometry, box):
    """Print the envelope and how the arm's rest poses sit in it."""
    size = [high - low for low, high in zip(box['min'], box['max'])]
    for i, axis in enumerate(AXES):
        print(f'  {axis}: {box["min"][i]:+.3f} to {box["max"][i]:+.3f} m   ({size[i]:.3f} m)')
    path = rest_path(JOINTS, SLEEP)
    for name, q in (('home', path[0]), ('sleep', path[-1])):
        outside, axis, side, link = excess(geometry, box, q)
        where = f'{link.split("/")[-1]}, {axis} {side}'
        if outside > 0:
            print(f'  {name} pose: OUTSIDE by {outside * 1000:.0f} mm ({where})')
        else:
            print(f'  {name} pose: inside, nearest side {-outside * 1000:.0f} mm away ({where})')


def finish(geometry, box, points, padding, out, confirm):
    """Pad the box, make it contain the rest poses, show it, and save it."""
    box = {'min': [v - padding for v in box['min']], 'max': [v + padding for v in box['max']]}
    box, moved = grow_to_contain(geometry, box, rest_path(JOINTS, SLEEP))
    box = checked_envelope(box)
    print('\nWork envelope (base frame):')
    describe(geometry, box)
    if moved:
        print(f'  Enlarged on {", ".join(moved)} to contain the arm at home, at sleep and in '
              'between.')
    if confirm and ask(f'\nSave to {out}? y to save, anything else to discard: ') != 'y':
        print('Not saved.')
        return
    print(f'Saved: {save_envelope(box, out, points)}')
    print('Supervised scripts started from now on use it.')


def show(args):
    box = load_envelope(args.file)
    if box is None:
        print(f'No work envelope is set ({args.file} does not exist).')
        return
    print(f'Work envelope in {args.file} (base frame):')
    describe(RobotGeometry(generate_urdf(ROBOT_MODEL, ROBOT_MODEL)), box)


def set_box(args):
    box = checked_envelope({'min': args.min, 'max': args.max}, '--min/--max')
    geometry = RobotGeometry(generate_urdf(ROBOT_MODEL, ROBOT_MODEL))
    finish(geometry, box, [args.min, args.max], args.padding, args.file, confirm=False)


def clear(args):
    if os.path.exists(args.file):
        os.remove(args.file)
        print(f'Removed {args.file}: no work envelope is set.')
    else:
        print(f'No work envelope is set ({args.file} does not exist).')


def teach(args):
    # imported here so that show, set and clear work without the Interbotix modules loaded
    from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
    from xsarm_scripts.supervisor import SupervisedManipulator

    if not INTERACTIVE and args.file == envelope_path():
        sys.exit('Without a terminal, teach only writes to a file named with --file.')
    bot = SupervisedManipulator(robot_model=ROBOT_MODEL, work_envelope=None)
    robot_startup()
    core = bot._bot.core  # torque switching is deliberately not offered by the supervisor
    geometry = RobotGeometry(generate_urdf(ROBOT_MODEL, ROBOT_MODEL))
    tip = f'{ROBOT_MODEL}/ee_gripper_link'
    points = []
    try:
        if ask('\nHOLD THE ARM by the forearm and the gripper: it drops when released.\n'
               'Enter to switch the torque off, q to cancel: ') == 'q':
            return
        core.robot_torque_enable('group', 'arm', False)
        try:
            while True:
                answer = ask(f'  [{len(points)} recorded] Carry the gripper tip to a corner of '
                             'the space to allow. Enter to record it, d when done: ')
                if answer == 'd' or (not INTERACTIVE and len(points) >= 2):
                    break
                joints = dict(zip(JOINTS, bot.arm.get_joint_positions()))
                point = [float(v) for v in geometry.link_transforms(joints)[tip][:3, 3]]
                points.append(point)
                print('    recorded x = {:+.3f}, y = {:+.3f}, z = {:+.3f} m'.format(*point))
            ask('  Bring the arm to a pose it can hold, clear of everything. Enter to switch '
                'the torque back on, then let go: ')
        finally:
            core.robot_torque_enable('group', 'arm', True)
            time.sleep(0.5)
            bot.arm.capture_joint_positions()
    finally:
        robot_shutdown()
    if not points:
        print('No points recorded; the envelope is unchanged.')
        return
    box = {'min': [min(p[i] for p in points) - 1e-4 for i in range(3)],
           'max': [max(p[i] for p in points) + 1e-4 for i in range(3)]}
    finish(geometry, box, points, args.padding, args.file, confirm=INTERACTIVE)


def main():
    parser = argparse.ArgumentParser(description='Set up the work envelope of the arm.')
    parser.add_argument('--file', default=envelope_path(),
                        help='envelope file (default: the one supervised scripts load)')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('show', help='print the envelope').set_defaults(run=show)
    commands.add_parser('clear', help='remove the envelope').set_defaults(run=clear)
    by_numbers = commands.add_parser('set', help='give two opposite corners as numbers')
    by_numbers.add_argument('--min', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'))
    by_numbers.add_argument('--max', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'))
    by_numbers.set_defaults(run=set_box)
    by_hand = commands.add_parser('teach', help='carry the gripper to the corners by hand')
    by_hand.set_defaults(run=teach)
    for command in (by_numbers, by_hand):
        command.add_argument('--padding', type=float, default=0.0,
                             help='distance [m] added on every side of the corners given')
    args = parser.parse_args()
    try:
        args.run(args)
    except ValueError as problem:
        sys.exit(str(problem))


if __name__ == '__main__':
    main()
