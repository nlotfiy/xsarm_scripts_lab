#!/usr/bin/env python3

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import numpy as np
from xsarm_scripts.supervisor import MotionRejected, SupervisedManipulator

# VX300S joint limits (degrees):
# waist: (-180, 180), shoulder: (-101, 101), elbow: (-101, 92),
# forearm_roll: (-180, 180), wrist_angle: (-107, 130), wrist_rotate: (-180, 180)

JOINT_ALIASES = {
    't1': 'waist',
    't2': 'shoulder',
    't3': 'elbow',
    't4': 'forearm_roll',
    't5': 'wrist_angle',
    't6': 'wrist_rotate',
}


def main():
    bot = SupervisedManipulator(robot_model='vx300s')
    robot_startup()
    try:
        bot.arm.go_to_home_pose()
        while True:
            joint_name = input('Enter joint [t1-t6]: ')
            angle_deg = float(input('Enter angle (degrees): '))
            angle_rad = np.deg2rad(angle_deg)
            try:
                bot.arm.set_single_joint_position(JOINT_ALIASES[joint_name], angle_rad)
            except MotionRejected as rejection:
                print(f'Not moved: {rejection}')
            print(bot.arm.T_sb)
    except KeyboardInterrupt:
        pass
    except MotionRejected as rejection:
        print(f'Stopped: {rejection}')
    try:
        bot.arm.go_to_home_pose()
        bot.arm.go_to_sleep_pose()
    except MotionRejected as rejection:
        print(f'Not parked: {rejection}')
    robot_shutdown()


if __name__ == '__main__':
    main()
