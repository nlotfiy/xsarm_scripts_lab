#!/usr/bin/env python3

import argparse
import sys
import time

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from xsarm_scripts.supervisor import MotionRejected, SupervisedManipulator
from xsarm_scripts.supervisor.collision import SelfCollisionModel
from xsarm_scripts.supervisor.geometry import generate_urdf, RobotGeometry

"""
Show the elbow range the self-collision check gives up (task 1.9 in docs/supervisor_steps.md).

The arm folds its elbow to three angles in turn and pauses at each, so they can be compared:

- 1.35 rad, the furthest the check allowed with the first capsule model;
- 1.53 rad, the furthest it allows now, with its 10 mm margin;
- 1.57 rad, the limit floor.

The shoulder is held back and the wrist up, so the gripper stays 76 mm or more above the base
plane.

For this demonstration only, the self-collision margin is lowered to zero so that the last
pose is sent; every other check of the supervisor stays on. Measured offline against the link
meshes, the links are 70 mm, 33 mm and 21 mm apart at the three poses.

Try it in simulation first:

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s use_sim:=true
    ros2 run xsarm_scripts elbow_range_demo
"""

SHOULDER, WRIST_ANGLE = -1.0, -1.0
POSES = [
    ('furthest the first capsule model allowed', 1.35),
    ('furthest the self-collision check allows now', 1.53),
    ('limit floor', 1.57),
]
# the capsules are 1.5 mm apart at the last pose; capsules that touch are still refused
DEMO_MARGIN = 0.0


def run(bot, repeats, pause, moving_time):
    arm = bot.arm
    names = arm.group_info.joint_names
    geometry = RobotGeometry(generate_urdf('vx300s', 'vx300s'))
    capsules = SelfCollisionModel.load(geometry, 'vx300s')
    arm.go_to_home_pose()
    for _ in range(repeats):
        for label, elbow in POSES:
            joints = [0.0, SHOULDER, elbow, 0.0, WRIST_ANGLE, 0.0]
            arm.set_joint_positions(joints, moving_time=moving_time)
            q = dict(zip(names, joints))
            clearance, link, other = capsules.clearance(q)
            x, _, z = geometry.link_transforms(q)['vx300s/ee_gripper_link'][:3, 3]
            print(f'elbow {elbow:.2f} rad ({label}): gripper at x = {x * 1000:.0f} mm, '
                  f'z = {z * 1000:.0f} mm; capsule clearance {clearance * 1000:+.0f} mm '
                  f'({link} to {other})', flush=True)
            time.sleep(pause)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()


def main():
    parser = argparse.ArgumentParser(
        description='Fold the elbow to 1.35, 1.53 and 1.57 rad in turn, repeatedly.')
    parser.add_argument('--repeats', type=int, default=3, help='times to go through the poses')
    parser.add_argument('--pause', type=float, default=4.0, help='time [s] held at each pose')
    parser.add_argument('--moving-time', type=float, default=3.0, help='time [s] of each move')
    args = parser.parse_args()

    bot = SupervisedManipulator(robot_model='vx300s', self_margin=DEMO_MARGIN)
    robot_startup()
    try:
        run(bot, args.repeats, args.pause, args.moving_time)
    except MotionRejected as rejection:
        bot.get_node().get_logger().error(f'Stopped: {rejection}')
        sys.exit(1)
    finally:
        robot_shutdown()


if __name__ == '__main__':
    main()
