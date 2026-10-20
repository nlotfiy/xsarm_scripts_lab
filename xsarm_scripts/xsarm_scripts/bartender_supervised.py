#!/usr/bin/env python3

import sys

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
import numpy as np
from xsarm_scripts.supervisor import MotionRejected, SupervisedManipulator

"""
The motion sequence of the Interbotix bartender demo, sent through the supervisor.

The moves are the same as in interbotix_xsarm_control/demos/bartender.py. This script is the
test case for the supervisor in the simulation harness:

    ros2 run xsarm_scripts sim_harness --script xsarm_scripts/bartender_supervised.py
"""


def run(bot):
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
    bot.gripper.release()
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.grasp()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.release()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()


def main():
    bot = SupervisedManipulator(robot_model='vx300s')
    robot_startup()
    try:
        run(bot)
    except MotionRejected as rejection:
        bot.get_node().get_logger().error(f'Stopped: {rejection}')
        sys.exit(1)
    finally:
        robot_shutdown()


if __name__ == '__main__':
    main()
